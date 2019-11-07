#include "discriminator/estimator_lib.h"

/* Funzione che assegna dei valori standard alle variabili */
void initStandardVar()
{
  nodeName = "KalmanAndOF";
  idFrameToSave = 1;
  alphaROI = 0;
  alphaBar = 0;
  dt = 0.033;
  sigmaXAcc = 50;
  sigmaYAcc = 50;
  sigmaXMeasureVel = 6.6667;
  sigmaYMeasureVel = 6.6667;
  sigmaXMeasurePos = 0.6667;
  sigmaYMeasurePos = 0.6667;
  kBrake = 0.04;
  xAcc = 0;
  yAcc = 0;
  numbTrackingNodes = 4;
  teamReady = true;
  firstMeasure = false;
  NewMeasure = false;
  opticalStima = false;
  thresholdCorners = 0.01;
  distanceCorners = 2;
  frameToReset = 50;
  frameToBrake = 60;
  is4StateKalman = true;
  debugDoneWindow = false;
}


/*Callback ROI: Funzione che salva i dati relativi al nodo della rete neurale nella variabile globale "ROI"*/
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& data_frame)
{
  int i = 0;
  for(std::vector<float>::const_iterator it = data_frame -> data.begin(); it != data_frame -> data.end(); ++ it){
    ROI[i] = * it;
    i++;
  }
}

/*Callback id Frame corrente: Funzione che salva nella variabile "currentId" l'id relativo del frame ricevuto dalla camera*/
void IdframeCallback (const std_msgs::Int32& Idframe)
{
  currentId = Idframe.data;
}

/*Callback immagine: Funzione che salva nella variabile globale frame il contenuto del topic su cui pubblica il nodo"\cameraInput"*/
void imageCallback(const position_estimation_pkg::imAndId::ConstPtr& imAndid_msg)
{
  frame = cv_bridge::toCvShare(imAndid_msg->camImage, imAndid_msg, "bgr8")->image;
  currentId = imAndid_msg->idImage;
}

/*Callback immagine:*/
void startCallback(const sensor_msgs::ImageConstPtr& msg){

}

/*Callback del booleano sulla prima misura: Funzione che salva nella variabile "firstMeasure" il valore logico letto dal 
  topic "/data_frame/first_measure" su cui scrive il nodo "\NeuralNetworkSim"*/
void isFirstMeasureCallback (const std_msgs::Bool& isFirst)
{
  firstMeasure = isFirst.data;
}

/*Callback del booleano sulla prima misura: Funzione che salva nella variabile "NewMeasure" il valore logico letto dal 
  topic "/data_frame/new_measure" su cui scrive il nodo "\NeuralNetworkSim"*/
void newMeasureCallback (const std_msgs::Bool& newm)
{
  NewMeasure = newm.data;
}

/*Callback del booleano sulla sincronizzazione iniziale dei nodi: Funzione che imposta il valore della variabile logica 
  "teamReady" a true se tutti i nodi sono pronti */
void trackingStatusCallback (const position_estimation_pkg::trackingStatus::ConstPtr& status_msg)
{
  if(teamReady) return;
  trackingStates[status_msg->trackingSystem_id] = status_msg->isReady;
  int ready_counter = 0;;
  for(auto robot : trackingStates) 
  {
    if(robot.second) ready_counter++; 
  }
  if (ready_counter== 4) 
  { 
    ROS_INFO_STREAM("KalmanAndOF: Team is ready. Let's move! "); 
    teamReady = true;
  }
}

/* Funzione che pubblica sul topic "/team_status" l'id del nodo, un time stamp e imposta a vero il valore della variabile
   "isReady". La struttura del messaggio è definita dal messaggio "trackingStatus.msgs" definito della cartella "msg".*/
void publishReadyStatus()
{
  position_estimation_pkg::trackingStatus status;
  status.header.stamp = ros::Time::now();
  status.trackingSystem_id = nodeName; 
  status.isReady = true;
  trackingStatusPub.publish(status);
  ROS_INFO_STREAM("Estimator published ready status " );  
}

/* Funzione di attesa per la sincronizzazione iniziale dei nodi. */
void waitForTeam() 
{ 
  ros::Rate loopRate(1);
  while (!teamReady)//Si resta nel ciclo while finchè tutti i nodi non hanno pubblicato il messaggio "isReady"
  {
    publishReadyStatus();
    ros::spinOnce();
    loopRate.sleep();
  } 
}

/* Valutazione del baricentro della ROI dai dati forniti dal nodo "/NeuralNetworkSim": Si suppone che il baricentro del drone
   si trovi in corrispondenza del baricentro della ROI*/
vector<float> barEvaluation(vector<float> ROIIN)
{
  vector<float> bar(2);
  bar[0] = w * ( ROIIN[3] + ROIIN[5]) / 2;
  bar[1] = h * ( ROIIN[2] + ROIIN[4]) / 2;
  return bar;
}

/* Funzione di verifica dei valori della ROI; Se i valori della ROI superano i bordi dell'immagine, essi vengono settati
   con i limiti della stessa. Restituisce quindi una ROI che è interamente contenuta nell'immagine. */
Rect verifyRect (Rect ROIrect)
{
  if (ROIrect.x <= 0) 
  {  
    ROIrect.x = 1;
  }   
  if (ROIrect.x >= w)
  { 
    ROIrect.x = w-1; 
  }
    if (ROIrect.y <= 0) 
  {  
    ROIrect.y = 1;
  }   
  if (ROIrect.y >= h)
  {
    ROIrect.y = h-1;
  }
  if ((ROIrect.x + ROIrect.width) >= w)
  { 
    //ROIrect.width = ROIrect.width - ((ROIrect.x + ROIrect.width) - w);
    ROIrect.x = w - ROIrect.width - 1;
  }
  if ((ROIrect.y + ROIrect.height) >= h)
  { 
    //ROIrect.height = ROIrect.height - ((ROIrect.y + ROIrect.height) - h);
    ROIrect.y = h - ROIrect.height - 1;
  }
  return ROIrect;
}

/* Funzione che restitusce un vettore nella forma di stato, in cui i valori della posizione sono settati con i valori 
   dell'ultima misura, e la velocità è nulla. Tale funzione è usata per resettare o inizializzare il filtro di Kalman*/
vector<float> barToState()
{
  if(is4StateKalman)
  {
    vector<float> lastState(4);
    lastState[0] = barEvaluation(ROI)[0];
    lastState[1] = 0;
    lastState[2] = barEvaluation(ROI)[1];
    lastState[3] = 0;
    return lastState;
  }
  else
  {
    vector<float> lastState(6);
    lastState[0] = barEvaluation(ROI)[0];
    lastState[1] = 0;
    lastState[2] = 0;
    lastState[3] = barEvaluation(ROI)[1];
    lastState[4] = 0;
    lastState[5] = 0;
    return lastState;
  }
}

/* Funzione che ritaglia il frame del video secondo i dati della ROI forniti dal nodo "/NeuralNetworkSim". Restituisce
   un'immagine contenente la porzione di frame identificata dalla variabile globale "ROINN"*/
Mat frameCropperFromROI (vector<float> ROIIN, Mat frame, int id)
{
  Rect ROIrect;
  Mat frameCrop;

  
  ROIrect.x = (int) (w * ROIIN[3]);
  ROIrect.y = (int) (h * ROIIN[2]);
  ROIrect.width = (int) (w * (ROIIN[5] - ROIIN[3]));
  ROIrect.height = (int) (h * (ROIIN [4] - ROIIN [2]));
  ROIrect = verifyRect ( ROIrect);
  frameCrop = frame(ROIrect);
  if(saveFrameCut)
    imwrite(nameUser + "FLAVIO&JACOPO/jpgVideoROI/" + std::to_string(id)+ ".jpg", frameCrop);
  return frameCrop; 
  Mat frameElab;
  GaussianBlur(frameCrop, frameElab, Size(3,3), 0, 0);		//Size (5,5) buono ancora, un pò al limite
  
  //imwrite("/home/flavio/FLAVIO&JACOPO/jpgVideoROI/" + std::to_string(id)+ ".jpg", frameElab);
  return frameElab;
  
}

/* Funzione che ritaglia il frame del video secondo i dati del baricentro in ingresso alla funzione. Restituisce
   un' immagine contenente la porzione di frame la cui larghezza e altezza sono quelle identificate dall'ultimo dato 
   di misura ("ROINN"), ingrandite di un fattore "alpha" rispetto a quest'ultima. La posizione sul piano immagine è 
   definita dall baricentro in ingresso*/												//FJ modifica
Mat frameCropperFromBar (vector<float> ROIIN, Mat frameToCrop, vector<float> bar, int id)
{
  Rect ROIrec;
  Mat framecrop, detected_edges;
  // Vertice alto a sx della ROI, valore x
  ROIrec.x = (int) ((bar[0] - w * (ROIIN[5] - ROIIN[3]) / 2) - alphaBar * w * (ROIIN[5] - ROIIN[3]) / 2);
  // Vertice alto a sx della ROI, valore y
  ROIrec.y = (int) ((bar[1] - h * (ROIIN[4] - ROIIN[2]) / 2) - alphaBar * h * (ROIIN[4] - ROIIN[2]) / 2);
  // larghezza ROI
  ROIrec.width = (int)  (w * (ROIIN[5] - ROIIN[3]) + alphaBar * w * (ROIIN[5] - ROIIN[3]));
  // altezza ROI
  ROIrec.height = (int) (h * (ROIIN[4] - ROIIN[2]) + alphaBar * h * (ROIIN[4] - ROIIN[2])); 
  // Verifica che la ROI sia contenuta nel piano immagine
  ROIrec = verifyRect ( ROIrec);
  framecrop = frameToCrop(ROIrec);
  if(saveFrameCut)
    imwrite(nameUser + "FLAVIO&JACOPO/jpgVideoROI/" + std::to_string(id)+ ".jpg", framecrop);
  return framecrop;
  
  Mat frameElab;
  GaussianBlur(framecrop, frameElab, Size(3,3),0,0);
  //imwrite("/home/flavio/FLAVIO&JACOPO/jpgFrameSave/" + std::to_string(id) + ".jpg", frameElab);
  return frameElab;
}


/* Funzione che calcola i corners individuati nel frame di misura in quello attuale*/
vector<Point2f> newCorners ( Mat framemeasuregray, Mat framegray, std::vector<Point2f> cornersmeasure)
{
  // Output della funzione
  std::vector<Point2f> corners;
  // Memorizzo i corners individuati nel frame precedente
  oldstatus = newstatus;				
  // Calcolo Optical Flow discreto (Lukas Kanade)
  calcOpticalFlowPyrLK(framemeasuregray, framegray, cornersmeasure, corners , newstatus, err, winSize, 3, termcrit, 0, 0.001);   
  //Stampa dei corners individuati
  /*for (int i = 0; i < corners.size() ; i++)
  {
    circle( framegray, corners[i], 3, Scalar(255,0,0), -1, 8);
  }*/
  return corners;
}

/* Funzione che calcola il nuovo baricentro, a partire dai dati del vecchio e dai corners attuali;
   N.B. Attualmente non utilizzata poichè la correzione con il filtro di Kalman avviene con la misura della velocità, 
   calcolata dalla funzione "newVel".*/
vector<float> newBar ( std::vector<Point2f> cornersold, std::vector<Point2f> corners, std::vector<float> oldbar)
{
  std::vector<float> bar (2);
  float sumx = 0 ,sumy = 0;
  int index = 1;
  for (int i = 0; i <corners.size(); i++)
  {
    // Se lo stesso corner è presente sia nel frame attuale che nel precedente ne calcolo lo spostamento;
    if ( oldstatus [i] == 1 && newstatus[i] == 1)
    {
      sumx  = sumx + (corners[i].x - cornersold[i].x);
      sumy  = sumy + (corners[i].y - cornersold[i].y);
      index++;
    }
  }
  // Calcolo del nuovo baricentro.
  bar[0] = oldbar [0] + sumx / index;
  bar[1] = oldbar [1] + sumy / index;
  return bar;
}


vector<float> newVel ( std::vector<Point2f> cornersold, std::vector<Point2f> corners)
{
  std::vector<float> vel (2);
  float sumx = 0 ,sumy = 0;
  std::vector<float> centerCorn(2);
  int index = 1;
  for (int i = 0; i <corners.size(); i++)
  {
    // Se lo stesso corner è presente sia nel frame attuale che nel precedente ne calcolo la velocità di spostamento;
    if ( oldstatus [i] == 1 && newstatus[i] == 1)
    {
      sumx  = sumx + (corners[i].x - cornersold[i].x) / dt;
      sumy  = sumy + (corners[i].y - cornersold[i].y) / dt;
      index++;
    }
  }
  vel[0] = sumx / index;
  vel[1] = sumy / index;
  return vel;
}


/* Funzione di assegnamento alle variabili globali dei paramentri specificati nel launch file
   N.B. In alcuni casi, se nel launch file non viene specificato il paramentro, la variabile assume un valore di 
   default*/
void initFromLaunch(ros::NodeHandle nodeToInit)
{
  nodeToInit.getParam("/USER", nameUser);			//si legge da launch il path dell'user
  nodeToInit.getParam("/img_w", w);				//si legge da launch file larghezza immagine
  nodeToInit.getParam("/img_h", h);				//si legge da launch file altezza immagine
  nodeToInit.getParam("/nodes_freq", frequency);		//si legge da launch la frequenza del nodo
  
  //Variabili OF
  nodeToInit.getParam("/numbFramesOF", nFrameOptical);		//si legge da launch il numero di frame su cui applicare OF
  nodeToInit.getParam("/numbCorners", nCorners);		//si legge da launch il numero di feature che si usano per OF
  nodeToInit.getParam("/thresholdCorners", thresholdCorners);	//si legge da launch la soglia nella ricerca dei corners per l'OF
  oldstatus.resize(nCorners);					//viene inizializzata la lunghezza del vettore "oldstatus"
  newstatus.resize(nCorners);					//viene inizializzata la lunghezza del vettore "newstatus"
  nodeToInit.getParam("/thresholdAreaROIforDistCorners1", range1);	//si legge da launch il valore della soglia inferiore per l'assegnazione della distanza minima in ricerca corners
  nodeToInit.getParam("/thresholdAreaROIforDistCorners2", range2);	//si legge da launch il valore della soglia superiore per l'assegnazione della distanza minima in ricerca corners
  nodeToInit.getParam("/distanceBetweenCorners1", dist1);		//si legge da launch il valore distanza minima da assegnare a goodFeatureToTrack
  nodeToInit.getParam("/distanceBetweenCorners2", dist2);		//si legge da launch il valore distanza media da assegnare a goodFeatureToTrack
  nodeToInit.getParam("/distanceBetweenCorners3", dist3);		//si legge da launch il valore distanza massima da assegnare a goodFeatureToTrack
  
  //Variabili Kalman
  nodeToInit.getParam("/dt", dt);				//si legge da launch il valore da assegnare a "dt"
  nodeToInit.getParam("/sigmaXAcc", sigmaXAcc);			//si legge da launch il valore da assegnare a "sigmaXAcc"
  nodeToInit.getParam("/sigmaYAcc", sigmaYAcc);			//si legge da launch il valore da assegnare a "sigmaYAcc"
  nodeToInit.getParam("/sigmaXMeasureVel", sigmaXMeasureVel);	//si legge da launch il valore da assegnare a "sigmaXMeasureVel"
  nodeToInit.getParam("/sigmaYMeasureVel", sigmaYMeasureVel);	//si legge da launch il valore da assegnare a "sigmaYMeasureVel"
  nodeToInit.getParam("/sigmaXMeasurePos", sigmaXMeasurePos);	//si legge da launch il valore da assegnare a "sigmaXMeasurePos"
  nodeToInit.getParam("/sigmaYMeasurePos", sigmaYMeasurePos);	//si legge da launch il valore da assegnare a "sigmaYMeasurePos"
  nodeToInit.getParam("/xAcc", xAcc);				//si legge da launch il valore da assegnare a "xAcc"
  nodeToInit.getParam("/yAcc", yAcc);				//si legge da launch il valore da assegnare a "yAcc"
  nodeToInit.getParam("/kBrake", kBrake);			//si legge da launch il valore da assegnare a "kBrake"
  nodeToInit.getParam("/kBrake_vel", kBrake_vel);		//si legge da launch il valore da assegnare a "kBrake_vel"
  nodeToInit.getParam("/kBrake_acc", kBrake_acc);		//si legge da launch il valore da assegnare a "kBrake_acc"
  nodeToInit.getParam("/alphaROI", alphaROI);				//si legge da launch il valore da assegnare a "dt"
  nodeToInit.getParam("/alphaBar", alphaBar);				//si legge da launch il valore da assegnare a "dt"
  nodeToInit.getParam("/frameToReset", frameToReset);			//si ledde da launch il valore del numero di frames per il resettare
  nodeToInit.getParam("/frameToBrake", frameToBrake);			//si legge da launch il valore del numero di frame prima che avvenga la frenata
  
  //Variabili debug e dati statistici
  nodeToInit.getParam("/nameFileSaveTime", nameFileToSaveTime);	//si legge da launch il valore da assegnare al "nameFileToSaveTime"
  nodeToInit.getParam("/nameFileSaveKalman", nameFileToSaveKalman);	//si legge da launch il valore da assegnare al "nameFileSaveKalman"
  nodeToInit.getParam("/nameFileSaveCorners", nameFileToSaveCorners);	//si legge da launch il valore da assegnare al "nameFileSaveCorners
  nodeToInit.getParam("/nameFileSaveKalmanRecupero", nameFileToSaveKalmanRecupero);	//si legge da launch il valore da assegnare al "nameFileSaveKalmanRecupero
  nodeToInit.getParam("/saveFrameCut", saveFrameCut);			//si legge da launch il valore da assegnare al booleano di salvataggio dei frame ritagliati
  nodeToInit.getParam("/saveStatisticData", saveStatisticData);		//si legge da launch il valore da assegnare al booleano di salvataggio dei dati statistici
  nodeToInit.getParam("/debugDroneWindow", debugDoneWindow);		//si legge da launch il valore del booleano utilizzato per la generazione e plot del drone centrato
  nodeToInit.getParam("/typeOfKalmanModel", is4StateKalman);		//si legge da launch il valore del booleano utilizzato per determinare la tipologia del modello implementato in Kalman
}

/* Funzione che ritorna il segno del valore("val") in ingresso*/
float getSign(float val)											// Modifica per via dei 0.0 e 0
{
  if(val > 0.0)
    return 1.0;
  if(val < 0.0)
    return -1.0;
  if(val == 0.0)
    return 0.0;
}

/* Funzione che calcola l'accelerazione da dare in ingresso al sistema, in accordo con la direzione della velocità stimata*/
vector<float> evaluateAcceleration(vector<float> state)
{
  vector<float> acceleration(2);
//  acceleration[0] = getSign(state[1])*xAcc;
//  acceleration[1] = getSign(state[3])*yAcc;
  acceleration[0] = 0;
  acceleration[1] = 0;
  return acceleration;
}

/* Funzione che assegna la distanza minima in ricerca dei corner tramite un feedback di grandezza della ROI, i parametri saranno inseriti e settabili tramite launch
   è una funzione di test*/
void calDistanceBetweenCorners(vector<float> ROIIN)
{
  double lROI = w * (ROIIN[5] - ROINN[3]);
  double hROI = h * (ROIIN[4] - ROINN[2]);
  double areaROI = lROI * hROI;
  if(areaROI <= range1)
    distanceCorners = dist1;
  else
  {
    if(areaROI <= range2)
      distanceCorners = dist2;
    else
      distanceCorners = dist3;
  }
}


