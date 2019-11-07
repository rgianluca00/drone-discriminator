/*header file definisce la classe che implementa le funzionalità del filtro di Kalman.*/
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "opencv2/opencv.hpp"
#include "math.h"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

extern bool is4StateKalman;
typedef Matrix<float,6,6> Matrix6f;

class kalmanFilterVideo
{
  /* ---------------------------------Variabili del filtro di Kalman---------------------------------*/
  // Tempo di campionamento
  float dt;
  // Vettore contenente la varianza(simgmaX e sigmaY) dell' accelerazione in ingresso al sistema. 
  MatrixXf stateCovariance;
  // Vettore contenente la varianza(simgmaX e sigmaY) dell' errore della posizione in misura. 
  MatrixXf measurePosCovariance;
  // Vettore contenente la varianza(simgmaX e sigmaY) dell' errore della velocità in misura. 
  MatrixXf measureVelCovariance;
  // Vettore contenente lo stato iniziale del sistema
  MatrixXf initialState;
  // Struttura contenente le matrici utilizzate dal filtro di Kalman
  struct kalmanStruct
  {
    MatrixXf A;
    MatrixXf B;
    MatrixXf Cpos;
    MatrixXf Cvel;
    MatrixXf P;
    MatrixXf Q;
    MatrixXf Rpos;
    MatrixXf Rvel;
    MatrixXf state;
  } kalmanVariables;
  // Struttura contenente i dati delle misure passate. 
  // Essa è pensata per il riaggancio al frame attuale in caso di misura in ritardo. 
  struct oldParams
  {
    // matrice che conserva lo stato del relativo al passo identificato da "idOld"
    MatrixXf stateOld;
    // matrice che conserva la matrice P relativa al passo identificato da "idOld"
    MatrixXf POld;
    // matrice che conserva il frame relativo al passo identificato da "idOld"
    Mat frameOld;
    // id del passo salvato
    int idOld;
  };

  
  public:
    // Costruttore della classe
    kalmanFilterVideo (float samplingTime , MatrixXf sC , MatrixXf mPC  , MatrixXf mVC, MatrixXf m)
    {
      // Inizializzazione dei parametri di Kalman
      dt = samplingTime;
      stateCovariance = sC;
      measurePosCovariance = mPC;
      measureVelCovariance = mVC;
      initialState = m;
    }
    
    //Metodi della classe
    void initializeFilter ();
    
    vector<float> kalmanEstimation (vector<float> uv);
    
    vector<float> kalmanCorrectionPosition (vector<float> uv , vector<float> measure);
    
    vector<float> kalmanCorrectionVelocity(vector< float > uv, vector< float > measurev);
    
    void resetState (vector<float> state);
    
    vector<float> getState ();
    
    void initializeTimeMachine(Mat frame, int id);
    
    void insertParams(Mat frame, int id);
    // Vettore che conserva i parametri passati
    vector <oldParams> timeMachine;
    
    int useTimeMachine (int Id);
};

  
    
    
  
