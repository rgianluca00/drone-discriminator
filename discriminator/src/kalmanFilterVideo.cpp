#include "discriminator/kalmanFilterVideo.h"


using namespace std;
using namespace Eigen;

// Inizializzazione filtro di Kalman
void kalmanFilterVideo::initializeFilter()
{
  if(is4StateKalman)
  {		// ---------------------------------- Modello a 4 stati di Kalman ---------------------- // 
    // Matrice di stato
    kalmanVariables.A.resize(4,4);
    kalmanVariables.A << 1, dt, 0,0, 0, 1, 0,0, 0,0,1,dt,0,0,0,1;
    // Matrice degli ingressi
    kalmanVariables.B.resize(4,2);
    //kalmanVariables.B << 0.5*dt*dt, 0, dt, 0, 0, 0.5*dt*dt, 0, dt;
    kalmanVariables.B << 0,0, 0,0, 0,0, 0,0;
    // Matrice d'uscita(posizione)
    kalmanVariables.Cpos.resize(2,4);
    kalmanVariables.Cpos << 1,0,0,0,0,0,1,0;
    // Matrice d'uscita(velocità)
    kalmanVariables.Cvel.resize(2,4);
    kalmanVariables.Cvel << 0,1,0,0,0,0,0,1;
    // Matrice Q
    kalmanVariables.Q.resize(4,4);
    float q11 = dt*dt*dt*pow(stateCovariance(0,0),2)/3.0;
    float q12 = dt*dt*pow(stateCovariance(0,0),2)/2.0;
    float q21 = dt*dt*pow(stateCovariance(0,0),2)/2.0;
    float q22 = dt*pow(stateCovariance(0,0),2);
    
    float q33 = dt*dt*dt*pow(stateCovariance(0,1),2)/3.0;
    float q34 = dt*dt*pow(stateCovariance(0,1),2)/2.0;
    float q43 = dt*dt*pow(stateCovariance(0,1),2)/2.0;
    float q44 = dt*pow(stateCovariance(0,1),2);
    kalmanVariables.Q << q11,q12,0,0, q21,q22,0,0, 0,0,q33,q34, 0,0,q43,q44;
    
    // Matrice P
    kalmanVariables.P.resize(4,4);
    kalmanVariables.P = kalmanVariables.Q;
    // Matrice R(velocità)
    kalmanVariables.Rvel.resize(2,2);
    kalmanVariables.Rvel << pow(measureVelCovariance(0,0),2), 0,0,pow(measureVelCovariance (0,1),2 );
    // Matrice R(posizione)
    kalmanVariables.Rpos.resize(2,2);
    kalmanVariables.Rpos << pow(measurePosCovariance(0,0),2), 0,0,pow(measurePosCovariance (0,1),2 );
    // Stato del sistema
    kalmanVariables.state = initialState;
  }
  else
  {		// ---------------------------------- Modello a 6 stati di Kalman ---------------------- //
    // Matrice di stato
    kalmanVariables.A.resize(6,6);
    kalmanVariables.A << 1, dt,0.5*dt*dt,0,0,0, 0,1,dt,0,0,0, 0,0,1,0,0,0, 0,0,0,1,dt,0.5*dt*dt, 0,0,0,0,1,dt, 0,0,0,0,0,1;
    
    // Matrice degli ingressi
    kalmanVariables.B.resize(6,2);
    kalmanVariables.B << 0.5*dt*dt,0, dt,0, 1,0, 0,0.5*dt*dt, 0,dt, 0,1;
    // Matrice d'uscita(posizione) 
    kalmanVariables.Cpos.resize(2,6);
    kalmanVariables.Cpos << 1,0,0,0,0,0, 0,0,0,1,0,0;
    
    // Matrice d'uscita(velocità)
    kalmanVariables.Cvel.resize(2,6);
    kalmanVariables.Cvel << 0,1,0,0,0,0, 0,0,0,0,1,0;
    
    // Matrice Q
    kalmanVariables.Q.resize(6,6);
    
    float q11 = pow(stateCovariance(0,0),2)*pow(dt,5)/20.0;
    float q12 = pow(stateCovariance(0,0),2)*pow(dt,4)/8.0;
    float q13 = pow(stateCovariance(0,0),2)*pow(dt,3)/6.0;
    float q21 = pow(stateCovariance(0,0),2)*pow(dt,4)/8.0;
    float q22 = pow(stateCovariance(0,0),2)*pow(dt,3)/3.0;
    float q23 = pow(stateCovariance(0,0),2)*pow(dt,2)/2.0;
    float q31 = pow(stateCovariance(0,0),2)*pow(dt,3)/6.0;
    float q32 = pow(stateCovariance(0,0),2)*pow(dt,2)/2.0;
    float q33 = pow(stateCovariance(0,0),2)*dt;
    
    float q44 = pow(stateCovariance(0,1),2)*pow(dt,5)/20.0;
    float q45 = pow(stateCovariance(0,1),2)*pow(dt,4)/8.0;
    float q46 = pow(stateCovariance(0,1),2)*pow(dt,3)/6.0;
    float q54 = pow(stateCovariance(0,1),2)*pow(dt,4)/8.0;
    float q55 = pow(stateCovariance(0,1),2)*pow(dt,3)/3.0;
    float q56 = pow(stateCovariance(0,1),2)*pow(dt,2)/2.0;
    float q64 = pow(stateCovariance(0,1),2)*pow(dt,3)/6.0;
    float q65 = pow(stateCovariance(0,1),2)*pow(dt,2)/2.0;
    float q66 = pow(stateCovariance(0,1),2)*dt;
    
    kalmanVariables.Q << q11,q12,q13,0,0,0, q21,q22,q23,0,0,0, q31,q32,q33,0,0,0, 0,0,0,q44,q45,q46, 0,0,0,q54,q55,q56, 0,0,0,q64,q65,q66;
    
    // Matrice P
    kalmanVariables.P.resize(6,6);
    //kalmanVariables.P = kalmanVariables.B * kalmanVariables.Q * kalmanVariables.B.transpose();
    kalmanVariables.P = kalmanVariables.Q;
    
    // Matrice R(velocità)
    kalmanVariables.Rvel.resize(2,2);
    kalmanVariables.Rvel << pow(measureVelCovariance(0,0),2), 0,0,pow(measureVelCovariance (0,1),2 );
    
    // Matrice R(posizione)
    kalmanVariables.Rpos.resize(2,2);
    kalmanVariables.Rpos << pow(measurePosCovariance(0,0),2), 0,0,pow(measurePosCovariance (0,1),2 );
    
    // Stato del sistema
    kalmanVariables.state.resize(6,1);
    kalmanVariables.state = initialState;
  }

}


/* Inizializzazione del vettore contenente i parametri utili al riaggancio della stima al frame attuale*/
void kalmanFilterVideo::initializeTimeMachine(Mat frame, int id)
{
  oldParams tm;
  frame.copyTo(tm.frameOld);
  tm.idOld = id;
  tm.POld = kalmanVariables.P;
  tm.stateOld = kalmanVariables.state;
  /* La lunghezza del ciclo for determina la lunghezza del vettore timeMachine. Per modificare le dimensioni modificare la
     lunghezza del ciclo*/ 
  for (int i = 0; i < 30; i++)
    timeMachine.push_back(tm);
  cout << "Dimensione Time machine = " << timeMachine.size() << "\n";
}

/* Definizione del metodo della classe che consente di salvare le info relative a max 20 frames intermedi tra 
  l'ultima misura ricevuta e l'id attuale*/
void kalmanFilterVideo::insertParams(Mat frame, int id)
{
  oldParams tm;
  frame.copyTo(tm.frameOld);
  tm.idOld = id;
  tm.POld = kalmanVariables.P;
  tm.stateOld = kalmanVariables.state;
  timeMachine.push_back(tm);
  timeMachine.erase(timeMachine.begin());
}

/* Definizione del metodo della classe che consente di implementare la stima con il filtro di Kalman.*/
vector<float> kalmanFilterVideo::kalmanEstimation(vector<float> uv)
{
  vector<float> y(2);
  MatrixXf K(4,2);
  if(!is4StateKalman)
    K.resize(6,2);
  Matrix2f tmp;
  MatrixXf ym( 2,1);
  MatrixXf u(2,1);
  u(0) = uv[0];
  u(1) = uv[1];
  // P = A*P*A^(T) + B*Q*B^(T)
  kalmanVariables.P = kalmanVariables.A * kalmanVariables.P * kalmanVariables.A.transpose() + 
		      kalmanVariables.Q;
  // x(k+1) = A*x(k) + B*u(k)
  kalmanVariables.state = kalmanVariables.A * kalmanVariables.state + kalmanVariables.B * u;

  ym = kalmanVariables.Cpos * kalmanVariables.state;
  y[0] = ym(0);
  y[1] = ym(1);
  
  return y;
}

/* Definizionede del metodo della classe che consente di implementare la correzione sulla posizione della misura con 
   il filtro di Kalman*/
vector< float > kalmanFilterVideo::kalmanCorrectionPosition(vector< float > uv, vector< float > measurev)
{
   vector<float> y(2);
  MatrixXf K(4,2);
  if(!is4StateKalman)
    K.resize(6,2);
  Matrix2f tmp;
  MatrixXf ym( 2,1);
  MatrixXf u(2,1);
  MatrixXf measure(2,1);
  u(0) = uv[0];
  u(1) = uv[1];
  measure(0) = measurev[0];
  measure(1) = measurev[1];
  // P = A*P*A^(T) + B*Q*B^(T)
		      
  kalmanVariables.P = kalmanVariables.A * kalmanVariables.P * kalmanVariables.A.transpose() + 
		      kalmanVariables.Q;
  // x(k+1) = A*x(k) + B*u(k)
  kalmanVariables.state = kalmanVariables.A * kalmanVariables.state + kalmanVariables.B * u;
  // tmp = C*P*C^(T) + R
  tmp = kalmanVariables.Cpos * kalmanVariables.P * kalmanVariables.Cpos.transpose() + kalmanVariables.Rpos;
  // K = P*C^(T) * tmp^(-1) 
  K = kalmanVariables.P * kalmanVariables.Cpos.transpose() * tmp.inverse();
  // x(k+1) = x(k+1) + K * (z - C*x(k+1))    
  kalmanVariables.state = kalmanVariables.state + K * (measure - kalmanVariables.Cpos * kalmanVariables.state);
  // P = (I - K*C) * P
  if(is4StateKalman)
    kalmanVariables.P = (Matrix4f::Identity() - K * kalmanVariables.Cpos) * kalmanVariables.P;
  else
    kalmanVariables.P = (Matrix6f::Identity() - K * kalmanVariables.Cpos) * kalmanVariables.P;
  // y = C * x(k+1);
  ym = kalmanVariables.Cpos * kalmanVariables.state;
  
  y[0] = ym(0);
  y[1] = ym(1);
  
  return y;
}

/* Definizionede del metodo della classe che consente di implementare la correzione sulla velocità della misura con 
   il filtro di Kalman*/
vector< float > kalmanFilterVideo::kalmanCorrectionVelocity(vector< float > uv, vector< float > measurev)
{
  vector<float> y(2);
  MatrixXf K(4,2);
  Matrix2f tmp;
  MatrixXf ym( 2,1);
  MatrixXf u(2,1);
  MatrixXf measure(2,1);
  u(0) = uv[0];
  u(1) = uv[1];
  measure(0) = measurev[0];
  measure(1) = measurev[1];
  // P = A*P*A^(T) + B*Q*B^(T)
  
  kalmanVariables.P = kalmanVariables.A * kalmanVariables.P * kalmanVariables.A.transpose() + 
		      kalmanVariables.Q;
  // x(k+1) = A*x(k) + B*u(k)
  kalmanVariables.state = kalmanVariables.A * kalmanVariables.state + kalmanVariables.B * u;
  // tmp = C*P*C^(T) + R
  tmp = kalmanVariables.Cvel * kalmanVariables.P * kalmanVariables.Cvel.transpose() + kalmanVariables.Rvel;
  // K = P*C^(T) * tmp^(-1)
  K = kalmanVariables.P * kalmanVariables.Cvel.transpose() * tmp.inverse();
  // x(k+1) = x(k+1) + K * (z-C*x(k+1))   
  kalmanVariables.state = kalmanVariables.state + K * (measure - kalmanVariables.Cvel * kalmanVariables.state);
  // P = (I - K*C) * P
  if(is4StateKalman)
    kalmanVariables.P = (Matrix4f::Identity() - K * kalmanVariables.Cvel) * kalmanVariables.P;
  else
    kalmanVariables.P = (Matrix6f::Identity() - K * kalmanVariables.Cvel) * kalmanVariables.P;
  // y = C * x(k+1);
  ym = kalmanVariables.Cpos * kalmanVariables.state;
  y[0] = ym(0);
  y[1] = ym(1);
  
  return y;
}

/* Definizione del metodo della classe che restituisce lo stato attuale del sistema*/
vector <float> kalmanFilterVideo::getState()
{
  if(is4StateKalman)
  {
    std::vector<float> state(4);
    state[0] = kalmanVariables.state(0);
    state[1] = kalmanVariables.state(1); 
    state[2] = kalmanVariables.state(2);
    state[3] = kalmanVariables.state(3);
    return state;
  }
  else
  {
    std::vector<float> state(6);
    state[0] = kalmanVariables.state(0);
    state[1] = kalmanVariables.state(1); 
    state[2] = kalmanVariables.state(2);
    state[3] = kalmanVariables.state(3);
    state[4] = kalmanVariables.state(4);
    state[5] = kalmanVariables.state(5);
    return state;
  }
}

/* Definizione del metodo della classe che setta lo stato del sistema in accordo con il vettore in ingresso
   alla funzione "state"*/
void kalmanFilterVideo::resetState(vector<float> state)
{
  if(is4StateKalman)
    kalmanVariables.state << state[0],state[1], state[2], state[3];
  else
    kalmanVariables.state << state[0],state[1], state[2], state[3],state[4],state[5];
}

/* Definizione del metodo della classe che riporta lo stato e la matrice P al passo indicato da "Id", in ingresso alla 
   funzione. Il metodo è usato per tenere conto dell'eventuale ritardo, dovuto alla computazione della rete neurale. 
   Questo metodo permette quindi di aggiornare la stima, in virtù del nuovo dato, consentendo di analizzare il frame a 
   partire dal passo identificato da "Id". Esso inoltre resetta lo stato e la matrice P ai valori che avevano assunto al passo "Id".
   L'output della funzione restituisce la posizione occupata nel vettore "timeMachine" dei dati relativi a "Id".*/
int kalmanFilterVideo::useTimeMachine (int Id)
{
  int sizeBuff = timeMachine.size();
  int index = sizeBuff+1;
  for (int i = 0; i < sizeBuff; i++)
  {
    if (timeMachine[i].idOld == Id)
    {
      index = i;						// si potrebbe mettere un brack per uscire prima dal ciclo (per la Jetson può risultare un vantaggio)
      //break;
    }
  }
  if (index == sizeBuff+1)
  {   
    //cout << "Misura Non trovata" << endl;
    return index;
  }
  kalmanVariables.P = timeMachine[index].POld;
  kalmanVariables.state = timeMachine[index].stateOld;
  return index;   
}
