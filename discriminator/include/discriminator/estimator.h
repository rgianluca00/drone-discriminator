/* Header del nodo estimator, in esso vi sono definite tutte le variabili utili per la gestione del nodo*/

#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__

#include <iostream>
#include <stdio.h>
#include <time.h>

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"				//Per scrivere messaggi di array di float a 32 bit
#include "std_msgs/Int32.h"					//Per scrivere messaggi di interi a 32 bit
#include <image_transport/image_transport.h>			//Per scrivere messaggi contenenti immagini
#include "std_msgs/Bool.h"					//Per scrivere messaggi di variabili booleane

#include "opencv2/opencv.hpp"					//Inlcusione delle funzioni della lib.OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"

#include "discriminator/trackingStatus.h"
#include "discriminator/estimator_lib.h"		//Inclusione della libreria di supporto
#include "discriminator/kalmanFilterVideo.h"		//Inclusione della libreria che definisce la classe che implementa le funzionalità del filtro di Kalman
#include "discriminator/imageAnalysis.h"		//Include della libreria che permette il salvataggio dei dati statistici

using namespace std;
using namespace cv;

//-----------------------------------------Variabili condivise dalle funzioni:-----------------------------------------//
int h;					// Altezza immagine(pixel, parametro letto da launch file)
int w;					// Ampiezza immagine(pixel, parametro letto da launch file)

//Parametri del nodo ROS
string nodeName;			// Stringa identificativa del nodo
double frequency;			// Frequenza del ciclo principale del nodo

//Variabili utilizzate nel calcolo dell' Optical Flow
Mat frame;				// Variabile di appoggio su cui salvare il frame corrente del nodo "\cameraInput".
int nFrameOptical;			// # frame successivi alla misura su cui effetuare Optical Flow
int nCorners;				// # di features del drone da identificare nella ROI
int idFrameToSave;

/*nei tipi richiesti dalla funzione di opencv "calcOpticalFlowPyrLK" */
vector<uchar> newstatus;		// Vettore di variabili che riporta l'individuazione dei corrispondenti corner nell'ultimo frame analizzato
vector<uchar> oldstatus;		// Vettore di variabili che contiene i valori di "newstatus" al frame precendente; serve per il confronto
vector<float> err;
Size winSize(5,5);
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

/* Dati della ROI: | #frame | prob. | Ymin | Xmin | Ymax | Xmax | */
vector<float> ROI(6);			// ROI della rete neurale
vector<float> ROINN(6);			// Variabile di appoggio per la ROI per il salvataggio del dato fornito della rete neurale, utilizzato quando non ho misure proveniente dalla rete stessa
double alphaROI;
double alphaBar;

/* Variabili Per il filtro di Kalman: Le seguenti variabili sono qui inizializzate con parametri di default, è possibile
   anche modificarne il valore da lounch file*/
float dt;			// tempo di campionamento del video(frame rate(Hz))

float sigmaXAcc;			// Varianza lungo x accelerazione(pixel)
float sigmaYAcc;			// Varianza lungo y accelerazione(pixel)

float sigmaXMeasureVel;	// Varianza dell'errore in misura della velocità(x) (pixel/s)
float sigmaYMeasureVel;	// Varianza dell'errore in misura della velocità(y) (pixel/s)

float sigmaXMeasurePos;	// Varianza dell'errore in misura della posizione (x) (pixel)
float sigmaYMeasurePos;	// Varianza dell'errore in misura della posizione (y) (pixel)

float kBrake, kBrake_vel, kBrake_acc;		// guadagno della retroazione(che genera azione frenante)
float xAcc;					// input di accelerazione (lungo x)
float yAcc;					// input di accelerazione (lungo y)

// Parametri di sincronozzazione dei nodi
ros::Subscriber trackingStatusSub;	// Sottoscrizione al topic per garantire sincronizzazione dei cicli
ros::Publisher trackingStatusPub;	// Pubblicazione al topic per garantire sincronizzazione dei cicli
map<string,bool> trackingStates;
int numbTrackingNodes;		// Numero di nodi complessivi dell'intera architettura ROS
bool teamReady;			// Variabile booleana che assume il valore vero quando tutti i nodi sono pronti

// Variabili delle callback
bool firstMeasure;		// Variabile che assume valore "true" se è arrivata la prima misura
bool NewMeasure;		// Variabile che assume valore "true" se è arrivata una nuova misura
int currentId;			// Id del frame corrente importato dal nodo camera
bool opticalStima;		// Variabile che assume valore "true" se si applica l'opticalFlow in fase di stima dopo aver rivecuto una misura

// Variabili di gestione della ricerca dei corner
double thresholdCorners;	// Variabile che assegna la threshold nella ricerca dei corner durante la fase di goodFeaturesToTrack
double distanceCorners;		// Variabile che definisce la distanza dei corners nella fase di ricerca nel goodFeaturesToTrack
double 	range1, range2;		// Variabili delle soglie nell'assegnamento della distanza minima durante la ricerca dei corners
int	dist1, dist2, dist3;	// Valori di ritorno delle distanze tra i corners nella funzione goodFeaturesToTrack

// Variabili di gestione dei salvataggi dei dati statistici ed immagini per debug
bool saveFrameCut;		// Booleano per salvare le foto ritagliate durante le fasi di recupero e OF
bool saveStatisticData;		// Booleano per salvare i dati statistici (kalman, temporali, corners)
std::string nameFileToSaveTime, nameFileToSaveKalman, nameFileToSaveCorners, nameFileToSaveKalmanRecupero;		// Nome del file dove salvare i dati statistici
std::string nameUser;		// Stringa contenente il path dell'utente, usato per rendere più generale il codice e di facile porting in jetson/altri pc
bool is4StateKalman;		// Booleano che mi indica che tipologia di modello si è usato: true == modello a 4 stati a velocità costante; false == modello a 6 stati a accelerazione costante
bool debugDoneWindow;		// Booleano utilizzato per la creazione e utilizzo di un'ulteriore finestra per debug del drone centrato


/*--------------Definizione Variabili private usate nel main--------------*/
// Varibili Grafiche
Mat frameCrop;					// Immagine RGB ritagliata secondo ROI
Mat frameCropGray;				// Immagine Gray ritagliata secondo ROI
Mat frameCropGrayOld;				// Immagine Gray del frame precedente ritagliata secondo ROI
Mat frameMeasureGray;				// Immagine Gray del frame di misura ritagliata secondo ROI
Mat frameToSave;				// Immagine Gray del frame corrente da salvare nella struttura dati "timeMachine"
vector<float> bar(2);
vector<float> vel(2);
vector<float> barMeasure(2);
  
// Variabili per il calcolo Optical Flow
vector <Point2f> cornersMeasure(nCorners);	// Corners del frame di misura
vector <Point2f> cornersold(nCorners);	// Corners del frame precedente
vector <Point2f> corners(nCorners);		// Corners del frame attuale
int opticalCounter;				// identifica il numero di frames successivi alla misura per OF(fase di riaggancio a id corrente)
int opticalCounterStima;			// identifica il numero di frames successivi alla misura per OF(fase succssiva al riaggancio)
int lC;						// Numero di corners individuati nel frame di misura
int indexTime;					// Indice corrispettivo del frame di cui è arrivata la misura
int frameToReset;				// Numero di frames senza misura dopo i quali resettare lo stato
int frameToBrake;				// Numero di frames dopo i quali attivare azione frenante
  
// Variabili temporali
struct timeval timeStart, timeEnd;		// Variabili temporali per analisi Real Time
double cpu_time_used;				// Tempo di esecuzione richiesto alla CPU
  

// Parametri del filtro di kalman
MatrixXf	stateCovariance (1,2);		// Varianza accelerazione (ax, ay)
MatrixXf 	measurePosCovariance (1,2);	// Varianza disturbo in misura (x, y)
MatrixXf 	measureVelCovariance (1,2);	// Varianza disturbo in misura (x, y)
MatrixXf 	initialState (4,1);		// Stato iniziale
 
vector<float> internalState (4);		// Variabile per salvare lo stato interno(utile alla frenatura)
vector<float> u(2);

// Dichiarazione dei messaggi in uscita
std_msgs::Bool FlagMeasure;
std_msgs::Float32MultiArray barOut;
std_msgs::Float32MultiArray RoiOut;

/* Variabili booleane per funzionamento macchina a stati, usate per riaccordarsi 
 * al frame corrente, utilizzando la stuttura "timeMachine"				*/
bool misura;					// Variabile che attiva la routine legate alla nuova misura
bool optical;					// Variabile che attiva la routine legate all'optical flow
bool estimation;				// Variabile che attiva la routine legate alla stima
bool actualMeasure;				// Variabile che segnala il riaggiornamento della stima al frame corrente
// Variabili per il reset dello stato, nel caso in cui non arrivi misura da troppo tempo
bool reset;					// Assume il valore "true" se la misura manca da molto tempo, resetta lo stato di kalman all'ultima misura
bool trustMeasure;				// Flag che segnala attendibilità della misura, attiva l'azione frenante.

#endif	//__ESTIMATOR_H__
