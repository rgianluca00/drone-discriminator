#ifndef __IMAGEANALYSIS_H_
#define __IMAGEANALYSIS_H_

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

extern bool is4StateKalman;

/* Definizione della classe che permette l'apertura, scrittura e chiusura di un file in cui vengono salvati i valori temporali d'esecuzione del nodo */
class timeFile
{
public:
  
  std::string nameFile;		// Definizione del nome del file assegnato
  ofstream stFile;		// Definizione dello stream del file
  bool flagNeuralData;		// Flag che rileva l'arrivo misura da Rete Neurale
  
  
  /* Costruttore della classe */
  timeFile(bool flagND, std::string nameTo)
  { 
    flagNeuralData = flagND;
    nameFile.assign(nameTo);
  }
  
  /* Funzione che apre il file su cui scrivere i dati temporali */
  void openFile();
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 0 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNotNeural(int nCountLoop, float timeNode);
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 1 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNeural(int nCountLoop, float timeNode);
  
  /* Funzione che chiude il file */
  void closeFile();
  
};


/* Definizione della classe che permette l'apertura, scrittura e chiusura di un file in cui vengono salvati tutti gli stati di Kalman */
class kalmanFile
{
public:
  
  std::string nameFile;		// Definizione del nome del file assegnato
  ofstream stFile;		// Definizione dello stream del file
  bool flagNeuralData;		// Flag che rileva l'arrivo misura da Rete Neurale
  int nCountNeural;		// Salvataggio id del dato ricevuto dalla Rete Neurale
  std::vector<float> stato;
  
  
  /* Costruttore della classe */
  kalmanFile(bool flagND, std::string nameTo)
  { 
    flagNeuralData = flagND;
    nameFile.assign(nameTo);
  }
  
  /* Funzione che apre il file su cui scrivere i dati temporali */
  void openFile();
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 0 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNotNeural(int nCountLoop);
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 1 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNeural(int nCountLoop);
  
  /* Funzione che chiude il file */
  void closeFile();
  
};


/* Definizione della classe che permette l'apertura, scrittura e chiusura di un file in cui vengono salvati tutti i corners */
class cornersFile
{
public:
  
  std::string nameFile;		// Definizione del nome del file assegnato
  ofstream stFile;		// Definizione dello stream del file
  bool flagNeuralData;		// Flag che rileva l'arrivo misura da Rete Neurale
  int nCountNeural;		// Salvataggio id del dato ricevuto dalla Rete Neurale
  std::vector<float> stato;
  
  
  /* Costruttore della classe */
  cornersFile(bool flagND, std::string nameTo)
  { 
    flagNeuralData = flagND;
    nameFile.assign(nameTo);
  }
  
  /* Funzione che apre il file su cui scrivere i dati temporali */
  void openFile();
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 0 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNotNeural(int nCountLoop, std::vector<Point2f> cornersS, int count);
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 1 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNeural(int nCountLoop, std::vector<Point2f> cornersS, int count);
  
  /* Funzione che chiude il file */
  void closeFile();
  
};



/* Definizione della classe che permette l'apertura, scrittura e chiusura di un file in cui vengono salvati tutti i kalman in recupero */
class kalmanRecuperoFile
{
public:
  
  std::string nameFile;		// Definizione del nome del file assegnato
  ofstream stFile;		// Definizione dello stream del file
  bool flagNeuralData;		// Flag che rileva l'arrivo misura da Rete Neurale
  int nCountNeural;		// Salvataggio id del dato ricevuto dalla Rete Neurale
  std::vector<float> stato;
  
  
  /* Costruttore della classe */
  kalmanRecuperoFile(bool flagND, std::string nameTo)
  { 
    flagNeuralData = flagND;
    nameFile.assign(nameTo);
  }
  
  /* Funzione che apre il file su cui scrivere i dati temporali */
  void openFile();
  
  /* Funzione che scrive nel file il valore temporale del passo i-esimo e il flag 1 se non si è ricevuto un dato dalla rete neurale*/
  void writeDataNeural(int idAttuale, int nCountLoop);
  
  /* Funzione che chiude il file */
  void closeFile();
  
};


#endif		//endif __IMAGEANALYSIS_H_