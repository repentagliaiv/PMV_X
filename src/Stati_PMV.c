/**********************************

 Maggio 2015
 *
 * Macchina degli stati del dispositivo PMV
 *
 * Il dispositivo prevede l'uso di più stati.
 *
 * Il principale è quello legato alla condizione di presenza
 * o assenza di connessione "logica" al FEP.
 * La condizione di connesso o non connesso prevede un
 * unico stato intermedio e temporaneo, attivo solo durante
 * l'avvio della connessione


 **********************************/

#include <xc.h>
#include <string.h>
#include <stdint.h>
#include "tick.h"
#include "pcf8583.h"
#include "Utili_InfraPMV.h"

#define DIM_PMV_INBUF   512
#define DIM_PMV_OUTBUF   512

// Tempo di permanenza dell'offline oltre il quale il display viene cancellato
#define TIMEOUT_CANCELLAZIONE   (10*TICK_SECOND)

#define TO_ATTESA_OK    (TICK_SECOND/5)     // L'attesa è sempre multiplo di 200ms

void Putc(unsigned char);
void Puth(unsigned char);
void Putl(unsigned long);
void Puti(unsigned int);
void Puts(const char* s);
void Putse(far char* s);

void Disconnetti_Client(void);
unsigned char Client_Connesso(void);

far char __section("myData") PMV_InBuffer[DIM_PMV_INBUF];
far char __section("myData") PMV_OutBuffer[DIM_PMV_OUTBUF]; // Buffer di uscita per il socket client
far char __section("myData") PMV_OutBuffer_Server[DIM_PMV_OUTBUF];  // Buffer di uscita per il socket server

// Buffer generico da utilizzare alla stregua di una variabile automatica
#define DIM_BUF_GEN 512
far char __section("myData") BufGen[DIM_BUF_GEN];

struct str_PacchettoPMV{
    char NUM_MSG[5];
    char COD_MITT[5];
    char COD_DEST[5];
    char RIF_NUM_MSG[5];
    char COMANDO[15];
    char PARAMETRI[1];
};

far struct str_PacchettoPMV *ComandoFEP = (far struct str_PacchettoPMV*)PMV_InBuffer;
far struct str_PacchettoPMV *RispostaFEP = (far struct str_PacchettoPMV*)PMV_OutBuffer;

enum errori {
    OK,
    CARATTERE_NON_VALIDO,
    DESTINATARIO_ERRATO,
    COMANDO_ERRATO,
    SINTASSI_ERRATA,
    DATI_NON_VALIDI,
    TIMEOUT_ECHO_FE,
    MESSAGGIO_NON_ATTESO,
    FINE_ERRORI,
    STARTED_DEV = FINE_ERRORI,
    STATO_DEV,
    ECHO_DEV
};

// Possibili messaggi di risposta non variabili
const char * Messaggi[] =
{
    "",
    "Carattere non valido\n",
    "Codice destinatario errato\n",
    "Comando errato\n",
    "Sintassi errata\n",
    "Dati non validi\n",
    "Timeout ECHO_FE\n",
    "Messaggio non atteso\n",
    "STARTED_DEV",
    "STATO_DEV",
    "ECHO_DEV"
};

enum daprocessare {
    cNESSUNO,
    cSTART_DEV,
    cECHO_FE,
    cESPONIprov,
    cESPONI
};

#define DIM_COMANDI_FEP 3
const char * ComandiFEP[] =
{
    "START_DEV",
    "ECHO_FE",
    "ESPONI"
};

const unsigned int LunghezzeMessaggi[] =
{
    42,
    58
};

unsigned int PosInbuf=0;
char MioID[] = {'C', 'T', 'E', 'S', 'T'};
char NumMsg[5];    // Campo NUM_MSG del comando ricevuto, da copiare nel
                   // campo RIF_NUM_MSG della relativa risposta
char CodMitt[5];

unsigned int NumRisp=0;

unsigned char ComandoDaProcessare=0;    // Se > 0 indica a Stato_Macchina() che
                                        // nel buffer c'è un comando da analizzare
unsigned int PeriodoEchoFEP;
unsigned int PeriodoEchoDEV = 90;
unsigned int PortaFEP;
unsigned char AllarmiPresenti = 0;
char ScrittaPagina[2][4][41];	// scritte da visualizzare

// CF sta per Connessione al FEP
enum StatiConnFEP{
    CF_NON_CONNESSO,
    CF_STARTED_DEV,
    CF_STATO_DEV,   // Secondo passo dell'avvio: invia STATO_DEV
    CF_CONNESSO
};

unsigned char Stato_CF = CF_NON_CONNESSO;

unsigned char AttesaOK = 0;
uint32_t     TimerAttesaOK, TimerEchoDEV, TimerEchoFEP, TimerCancellazione;
extern BOOL AvviaClient;

// Parsing del buffer di ricezione dalla posizione i per leggere un intero dec
static unsigned char LeggiInt(far unsigned char *i, unsigned int* temp)
{
    unsigned char n=0;
    *temp = 0;
    while(*i >= 0x0030 && *i <= 0x0039)
    {
        *temp *= 10;
        *temp += (unsigned int)((*i++)-0x30);
        n++;
    }
    return n;
}

// Scrive un int in decimale ad un indrizzo in far, usando 5 cifre
// allineate a dx e riempiendo con degli zeri a sx
// es. 345 -> 00345
void ScriviInt(far unsigned char *p, unsigned int i)
{
    *p++ = i / 10000 + 0x30;
    i %= 10000;
    *p++ = i / 1000 + 0x30;
    i %= 1000;
    *p++ = i / 100 + 0x30;
    i %= 100;
    *p++ = i / 10 + 0x30;
    i %= 10;
    *p = i + 0x30;
}

static void Messaggio_OUT(unsigned char messaggio, BOOL rif)
{
    unsigned char i;

    if(messaggio==0)
    {
        RispostaFEP->NUM_MSG[0] = 'O';
        RispostaFEP->NUM_MSG[1] = 'K';
        RispostaFEP->NUM_MSG[2] = '\n';
        RispostaFEP->NUM_MSG[3] = '\0';
    }
    else
    {
        ScriviInt(RispostaFEP->NUM_MSG, NumRisp++);
        memcpy(RispostaFEP->COD_MITT, MioID, 5);
        if(CodMitt[0])
            for(i=0;i<5;i++)
                RispostaFEP->COD_DEST[i] = CodMitt[i];
        else
            for(i=0;i<5;i++)
                RispostaFEP->COD_DEST[i] = ' ';

        // Sbianca campi RIF_NUM_MSG e COMANDO
        for(i=0;i<20;i++)
            RispostaFEP->RIF_NUM_MSG[i] = ' ';

        if(rif)
        {
            if(NumMsg[0])
                for(i=0;i<5;i++)
                    RispostaFEP->RIF_NUM_MSG[i] = NumMsg[i];
            else
                for(i=0;i<5;i++)
                    RispostaFEP->RIF_NUM_MSG[i] = ' ';
        }

        if(messaggio < FINE_ERRORI)
        {
            memcpy(RispostaFEP->COMANDO,"ERRORE",6);
            strcpy(&RispostaFEP->COMANDO[15],Messaggi[messaggio]);
        }
        else
        {
            memcpy(&RispostaFEP->COMANDO[0],Messaggi[messaggio],strlen(Messaggi[messaggio]));
        }
    }
}

void Invia_STARTED_DEV(void)
{
    Messaggio_OUT(STARTED_DEV, TRUE); // Intesta il messaggio
    ScriviInt(RispostaFEP->PARAMETRI, PeriodoEchoDEV);
    RispostaFEP->PARAMETRI[5] = ';';
    strcpy(&RispostaFEP->PARAMETRI[6],__DATE__ " " __TIME__ "\n");
    AttesaOK = 1;
Puts(" okstart ");
    AvviaClient=TRUE;
}

void Invia_STATO_DEV(void)
{
    unsigned char i,j;
    far char * p;

    Messaggio_OUT(STATO_DEV, FALSE); // Intesta il messaggio
    if(AllarmiPresenti)
        strcpy(&RispostaFEP->PARAMETRI[0],"ALLARME;");
    else
        strcpy(&RispostaFEP->PARAMETRI[0],"ONLINE;");
    p = strcat(&RispostaFEP->PARAMETRI[7],"PAGINA.1=\"");
    p+=10;
    for(i=0;i<4;i++)
    {
        for(j=0;j<40;j++)
        {
            *p = ScrittaPagina[0][i][j];
            if(*p==0)
                break;
            else
                p++;
        }
        *p++='"';
        *p++=',';
        *p++='"';
    }
    p -= 2;
    *p++ = ';';
    strcpy(p,"PAGINA.2=\"");
    p+=10;
    for(i=0;i<4;i++)
    {
        for(j=0;j<40;j++)
        {
            *p = ScrittaPagina[1][i][j];
            if(*p==0)
                break;
            else
                p++;
        }
        *p++='"';
        *p++=',';
        *p++='"';
    }
    p -= 2;
    *p++ = '\n';
    *p++ = '\0';

    AttesaOK = 1;
Puts(" okstat ");
    AvviaClient=TRUE;
}

void Invia_ECHO_DEV(void)
{
    Messaggio_OUT(ECHO_DEV, FALSE); // Intesta il messaggio
    RispostaFEP->PARAMETRI[0] = '\n';
    RispostaFEP->PARAMETRI[1] = 0;
    AttesaOK = 1;
Puts(" okeco ");
    AvviaClient=TRUE;
}

void Processa_ECHO_FEP(void)
{
    date_time_t dt;
    unsigned int t;

    // 6/2/12 All'arrivo di ECHO_FE sincronizza l'orologio interno
    LeggiInt(&ComandoFEP->PARAMETRI[2],&t);
    dt.year = (unsigned char)(t);
    LeggiInt(&ComandoFEP->PARAMETRI[5],&t);
    dt.month = (unsigned char)(t);
    LeggiInt(&ComandoFEP->PARAMETRI[8],&t);
    dt.day = (unsigned char)(t);
    LeggiInt(&ComandoFEP->PARAMETRI[11],&t);
    dt.hours = (unsigned char)(t);
    LeggiInt(&ComandoFEP->PARAMETRI[14],&t);
    dt.minutes = (unsigned char)(t);
    LeggiInt(&ComandoFEP->PARAMETRI[17],&t);
    dt.seconds = (unsigned char)(t);
    PCF8583_set_datetime(&dt);
}

// Copia una stringa passata in 'da' fino alle prime '"' processando gli eventuali
// '\\' e '\"' incontrati
far unsigned char* ProcessaRigaEsponi(far unsigned char *da, far unsigned char *a)
{
    unsigned char i;
    for(i=0;i<40;i++)
    {   // La riga può essere lunga al max 40 caratteri
        if(*da=='\\')
        {
            da++;
            if(*da=='\\')
                *a='\\';
            else if(*da=='"')
                *a='"';
            else
                return NULL;   // Errore, dopo '\' possonoe sserci solo '\' o '"'
        }
        else if(*da=='"')
        {   // Fine riga
            *a=0;
            return ++da;
        }
        else
            *a = *da;

        a++;
        da++;
    }
    // Se il processo arriva qui sono stati analizzati 40 caratteri. Verifica che
    // la stringa sia chiusa con un '"'
    if(*da=='"')
    {
        *a=0;
        return ++da;
    }
    else
        return NULL;
}

void Processa_ESPONI(void)
{
    unsigned char TempLumin;
    unsigned int TempTempoPagina[2], TempTempoLampeggio[2];
    far unsigned char *p;
    unsigned char Errore=0;
    unsigned char i;

    // Cerca PAGINA.1
    p = strstr(ComandoFEP->PARAMETRI, "PAGINA.1");
    if(p)
    {
        p+=8;
        if(*p++ != '=')
        {
            Errore = 1;
        }
        else
        {
            if(*p++ != '"')
            {
                Errore = 1;
            }
            else
            {
                for(i=0;i<4 && Errore==0;i++)
                {
                    p = ProcessaRigaEsponi(p,&BufGen[i*41]);
                    if(p == NULL)
                        Errore = 2;
                    else
                    {
                        // p punta al primo carattere dopo la '"' di fine riga
                        if(*p == ',')
                        {
                            p++;
                            if(*p++ != '"')
                            {
                                Errore = 1;
                            }
                        }
                        else if(*p != '\n' && *p != ';')
                            Errore = 3;
                        else
                            break;
                    }
                }
            }
        }
    }

    strncpy(&ScrittaPagina[0][0][0],&BufGen[0],40);
    strncpy(&ScrittaPagina[0][1][0],&BufGen[41],40);
    strncpy(&ScrittaPagina[0][2][0],&BufGen[82],40);
    strncpy(&ScrittaPagina[0][3][0],&BufGen[123],40);
}

void Stato_Macchina(void)
{
    unsigned int i;
    far unsigned char * p;
    uint32_t tempo;

    tempo = TickGet();

    if(AttesaOK>0)
    {
        if((tempo-TimerAttesaOK) > (25*TICK_SECOND))
        {   // In ogni caso non restare in questo stato troppo a lungo
    Puts("Massa\r\n");
            AttesaOK=0;
            AvviaClient=FALSE;
            Stato_CF = CF_NON_CONNESSO;
            ComandoDaProcessare = 0;
        }
        else
        if(AttesaOK%2==0)
        {
            if((tempo-TimerAttesaOK) > (TO_ATTESA_OK*AttesaOK))
            {   // Fai tre tentativi attendendo 200ms, 400ms e 600ms dopodiché
                // vai in errore
                if(Client_Connesso())
                {
    Puts("Disconno\r\n");
                    Disconnetti_Client();
                    AttesaOK++;
                    TimerAttesaOK=tempo;
                    if(AttesaOK==4)
                    {   // Atteso troppo -> ERRORE
                        AttesaOK=0;
                        AvviaClient=FALSE;
                        Stato_CF = CF_NON_CONNESSO;
                        ComandoDaProcessare = 0;
                    }
                }
                else
                {
                    // Se è passato troppo tempo in attesa dell'OK passa comunque in non connesso
                    if((tempo-TimerAttesaOK) > (5*TICK_SECOND))
                    {
    Puts("Disconn2\r\n");
                        AttesaOK=0;
                        AvviaClient=FALSE;
                        Stato_CF = CF_NON_CONNESSO;
                        CodMitt[0]=0;
                        NumMsg[0] = 0;
                        ComandoDaProcessare = 0;
                        TimerAttesaOK=tempo;
                    }
                }
            }
        }
        return;
    }

    if(Stato_CF == CF_NON_CONNESSO)
    {
        SpegniLedRosso();
        SpegniLedVerde();
        if(tempo-TimerCancellazione > TIMEOUT_CANCELLAZIONE)
        {
                    // TODO cancella display
        }

        if(ComandoDaProcessare==cSTART_DEV)
        {   // Ricevuto START_DEV. Analizzalo e, se valido, rispondi STARTED_DEV
            unsigned int j;
            unsigned char n;
            p = (far unsigned char *)ComandoFEP->PARAMETRI;
            n = LeggiInt(p, &i);
            p+=n+1;
            LeggiInt(p, &j);
            if(j>= 1024)
            {   // Messaggio OK
                PeriodoEchoFEP = i;
                PortaFEP = j;
                Messaggio_OUT(0, FALSE);   // OK
                Stato_CF = CF_STARTED_DEV;
                TimerEchoDEV = tempo;
            }
            else
                Messaggio_OUT(DATI_NON_VALIDI, TRUE);
        }
        else if(ComandoDaProcessare)
        {
            Messaggio_OUT(DATI_NON_VALIDI, TRUE);
        }
        ComandoDaProcessare = 0;
    }
    else if(Stato_CF == CF_STARTED_DEV &&
            PMV_OutBuffer[0] == 0)  // Se deve essere ancora inviato l'OK, aspetta. TODO: timeout?
    {
//        Puts("Invio started dev\r\n");
        Invia_STARTED_DEV();
        TimerAttesaOK=tempo;

        Stato_CF = CF_STATO_DEV;
    }
    else if(Stato_CF == CF_STATO_DEV)
    {
//        Puts("Invio stato dev\r\n");
        Invia_STATO_DEV();
        TimerAttesaOK=tempo;

        Stato_CF = CF_CONNESSO;

        TimerEchoFEP = tempo;
    }
    else if(Stato_CF == CF_CONNESSO &&
            PMV_OutBuffer[0] == 0)  // Se deve essere ancora inviato l'OK, aspetta. TODO: timeout?
    {
        if(tempo-TimerEchoDEV >= (PeriodoEchoDEV*TICK_SECOND))
        {
            TimerEchoDEV = tempo;
            Invia_ECHO_DEV();
            TimerAttesaOK=tempo;
    Puts("invio ecodev\r\n");
        }
        else if(ComandoDaProcessare==cECHO_FE)
        {   // Ricevuto ECHO_FE
    Puts("proceco\r\n");
            Messaggio_OUT(0, FALSE);   // OK

            Processa_ECHO_FEP();
            TimerEchoFEP = tempo;

        }
        else if(ComandoDaProcessare == cESPONIprov)
        {   // esponi
            Messaggio_OUT(0, FALSE);   // OK
            Processa_ESPONI();
            ComandoDaProcessare = cESPONI;
        }
        else if(ComandoDaProcessare == cESPONI)
        {   // esponi
            Invia_STATO_DEV();
            TimerAttesaOK=tempo;
            ComandoDaProcessare = 0;
        }
        else if(ComandoDaProcessare == cSTART_DEV)
        {   // FEP e PMV non sono allineati. Rimettiti offline
            Stato_CF = CF_NON_CONNESSO;
    Puts("riparto\r\n");
        }

        if(ComandoDaProcessare != cESPONI)
            ComandoDaProcessare = 0;
        // Verifica se è stato ricevuto l'ECHO_FE entro il tempo stabilito
        if(tempo-TimerEchoFEP >= (PeriodoEchoFEP*TICK_SECOND))
        {
            Messaggio_OUT(TIMEOUT_ECHO_FE,0);
            TimerCancellazione = tempo;
        }
    }
    else
    {
        // ERRORE: l'esecuzione non dovrebbe mai giungere qui.
    }
}

// Ricevuto un pacchetto completo, analizzalo e poni ComandoDaProcessare > 0 se
// è stato trovato un campo COMANDO noto
static unsigned char Processa_Inbuf(void)
{
    unsigned char i,j;

    // Verifica se il codice destinatario è corretto
    for(i=0; i<5; i++)
    {
        if(ComandoFEP->COD_DEST[i] != MioID[i])
        {
            Messaggio_OUT(DESTINATARIO_ERRATO, TRUE);
            return 1;
        }
    }

    // Analizza il campo COMANDO per capire quali azioni intraprendere
    for(i=0; i< DIM_COMANDI_FEP; i++)
    {
        if(PosInbuf<LunghezzeMessaggi[i])
            continue;

        for(j=0; j<15; j++)
        {
            if(ComandiFEP[i][j]==0) // Raggiunta fine stringa comando
            {   // => Comando trovato
                // Il resto del campo comando dev'essere riempito di spazi
                // NELL'IPOTESI che il pacchetto sia stato ricevuto completamente
                for(;j<15;j++)
                {
                    if(ComandoFEP->COMANDO[j] != ' ')
                    {
        Puts("campo sbagl ");
        Putc(ComandoFEP->COMANDO[j]);
        Puth(j);
        Puth(i);
                        Messaggio_OUT(DATI_NON_VALIDI, TRUE);
                        break;
                    }
                }
        Puts("com\r\n");
        Puth(j);
        Puth(i);
                ComandoDaProcessare = i+1; // E' stato trovato un comando

                // Salva i campi NUM_MSG e COD_MITT
                memcpy(NumMsg, ComandoFEP->NUM_MSG, 5);
                memcpy(CodMitt, ComandoFEP->COD_MITT, 5);

                return 0;
            }
            if(ComandoFEP->COMANDO[j] != ComandiFEP[i][j])
            {
        Puts("NOCcom\r\n");
//                Messaggio_OUT(COMANDO_ERRATO, TRUE);
                break;
            }
        }
    }

    return 2;
}

// Ricevuti dati sul socket server.
// Accodali e processali quando il pacchetto è completo
// Restituisci:
// null se non si deve inviare nessuna risposta (pacchetto non completo)
// oppure un puntatore al messaggio da inviare
void PMV_Dati_Rx(unsigned char *p, unsigned int q)
{
#define TIMEOUT_RX_PACCHETTO_ESPONI (2*TICK_SECOND)
#define TIMEOUT_RX_PACCHETTO (TICK_SECOND)

    static uint32_t TO_pacchetto;
    uint32_t tempo;

    tempo = TickGet();

    if(ComandoDaProcessare == cESPONIprov)
    {   // Solo per ESPONI: 
        if(tempo-TO_pacchetto > TIMEOUT_RX_PACCHETTO_ESPONI)
        {
            ComandoDaProcessare = 0;
            PosInbuf = 0;
        }
    }
    else
    {
        if(tempo - TO_pacchetto > TIMEOUT_RX_PACCHETTO)
        {
Puts("tempo scaduto\r\n");
            PosInbuf = 0;
        }
    }

    TO_pacchetto = tempo;

    for(;q>0;q--)
    {
        PMV_InBuffer[PosInbuf] = *p;
        PosInbuf++;

        if(*p == 0x0a)
        {
Puts("Ricio ");
Puth(PosInbuf);
Putc(PMV_InBuffer[20]);
Putc(PMV_InBuffer[21]);
Puth(Stato_CF);
Puth(AttesaOK);
            Processa_Inbuf();
//    if(ComandoDaProcessare != cESPONIprov && ComandoDaProcessare != 0)
    if(ComandoDaProcessare != 0)
    {
Puts("Cumande ja'");
Puth(ComandoDaProcessare);
        PosInbuf = 0;
    }
Puts("\r\n");
            return;
        }
        else if(*p < 32 || *p > 126)
        {   // Carattere non valido, invia ERRORE - Carattere non valido
Puts("Carattere non valido\r\n");
            PosInbuf=0;
            Messaggio_OUT(CARATTERE_NON_VALIDO, TRUE);
            return;
        }

        p++;

        if(PosInbuf==DIM_PMV_INBUF)
        {
            // Riempire il buffer significa perdere tutti i dati fino qui ricevuti
            // Occorre segnalazione?
Puts("buffer pieno\r\n");
            PosInbuf = 0;
        }
    }

    // NB: da specifiche Infracom ogni messaggio dovrebbe terminare con '\n' ma così
    // non è.
    //
    // Si effettua quindi il parsing del campo COMANDI dopo la ricezione di ogni singolo
    // pacchetto e, se si trova una stringa nota, si considera il paccheetto completo.
    //
    // NB!!!!!!!!!!!!!!!!!!
    // Questo è un problema nel caso un comando venga ricevuto in più pacchetti ed
    // il primo di questi contenga già, completo, il campo COMANDI.
    // Per ipotesi i pacchetti ricevuti dal FEP sono tutti abbastanza corti da essere
    // ricevuti in un unico giro del main() (e quindi un'unica chiamata allo stack TCP)
    //
    // Occorre però inserire un timeout in modo da:
    // - azzerare il puntatore del buffer PosInBuf in caso non sia ricevuto un pacchetto
    //   riconosciuto entro un certo tempo dall'inizio della ricezione
    // - considerare completa la ricezione dell'unico (? 25/5/15) comando potenzialmente
    //   lungo (ESPONI) solo dopo un certo tempo dall'inizio della ricezione

//    if(PosInbuf >=15)   // Inutile processare il messaggio se non sono stati ricevuti almeno 15 caratteri
//        Processa_Inbuf();
//    if(ComandoDaProcessare != cESPONIprov && ComandoDaProcessare != 0)
//    {
//Puts("Cumande ja'");
//Puth(ComandoDaProcessare);
//Puth(PosInbuf);
//Putc(PMV_InBuffer[20]);
//Putc(PMV_InBuffer[21]);
//Puth(Stato_CF);
//Puth(AttesaOK);
//Puts("\r\n");
//        PosInbuf = 0;
//    }

//    if(*p == (unsigned char)'\n' || PMV_InBuffer[PosInbuf-5] == (unsigned char)';')
//    {   // Ricezione messaggio completa, passa al processo
//        PMV_InBuffer[PosInbuf]='\n';    // BUG del FEP: il \n finale potrebbe non esserci
//        PosInbuf=0;
//        Processa_Inbuf();
//        return;
//    }
}