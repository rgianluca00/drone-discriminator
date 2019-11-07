#include "discriminator/estimator.h"			//Incusione dell'header dove sono definite le variabili

using namespace Eigen;

//---------------------------------------------- 	MAIN		---------------------------------------//

int main(int argc, char** argv)
{
  initStandardVar();
  ros::init(argc, argv, nodeName);		// Inizializzazione parametri ROS
  ros::NodeHandle node;
  initFromLaunch(node);				// Lettura dei parametri da launch file
  
  // Sottoscrizione ai topic:
  // - Lettura status nodi
  trackingStatusSub = node.subscribe("/team_status", 10, &trackingStatusCallback);
  // - Lettura valore ROI da /NeuralNetworkSim
  ros::Subscriber sub = node.subscribe("/data_frame/ROI", 1, chatterCallback);
  //Lettura frame da /cameraInput  
  //ros::Subscriber subCamera = node.subscribe("/cameraImage/Image", 1, imageCallback); 
  //Lettura del corrispettivo id associato al frame
  //ros::Subscriber subCameraProf = node.subscribe("/cameraImage/Idframe", 1, IdframeCallback);
  ros::Subscriber subImAndId = node.subscribe("/im_Id", 1, imageCallback);
  //Lettura flag di segnalazione nuova misura
  ros::Subscriber newMeasure = node.subscribe("/data_frame/new_measure", 1, newMeasureCallback);
  //Lettura flag segnalazione prima misura
  ros::Subscriber subIsFirstMeasure = node.subscribe("/data_frame/first_measure", 1, isFirstMeasureCallback);
  
  // Definizione dei topic su cui pubblicare:
  // Pubblicazione del baricentro associato all'ultimo frame
  ros::Publisher bar_pub = node.advertise<std_msgs::Float32MultiArray>("/Stampa/Bar", 1);
  // Pubblicazione della ROI associata all'ultimo frame
  ros::Publisher roi_pub = node.advertise<std_msgs::Float32MultiArray>("/Stampa/Roi", 1);
  // Pubblicazione dello status del nodo
  trackingStatusPub = node.advertise<position_estimation_pkg::trackingStatus>("/team_status", 10);
  // Pubblicazione flag segnalazione prima misura
  ros::Publisher pubIsNewMeasure = node.advertise<std_msgs::Bool>("/data_frame/new_measure",1);
  // Sottoscrizione topic immagine raw
  ros::Subscriber subCamera = node.subscribe("/camera/rgb/image_rect_color", 1, startCallback); 


  /*--------------Definizione Variabili private usate nel main--------------*/

  
  // Variabili dei file di salvataggio
  timeFile tFile(false, nameFileToSaveTime);				// Definizione della classe tFile, usata per salvare i dati temporali
  kalmanFile kFile(false, nameFileToSaveKalman);			// Definizione della classe kFile, usata per salvare i dati di kalman
  cornersFile cFile(false, nameFileToSaveCorners);			// Definizione della classe cFile, usata per salvare i corners
  kalmanRecuperoFile krFile(false, nameFileToSaveKalmanRecupero);	// Definizione della classe krFile, usata per salvare i dati degli stati di kalman nel recupero
  
  /*--------------Inizializzazione Variabili usate nel main--------------*/
  frame = Mat::zeros(cv::Size(w,h), CV_64FC1);
  frameCrop = Mat::zeros(cv::Size(w,h), CV_64FC1);
  frameToSave = Mat::zeros(cv::Size(w,h), CV_64FC1);
  frameCropGray = Mat::zeros(cv::Size(w,h), CV_64FC1);
  frameCropGrayOld = Mat::zeros(cv::Size(w,h), CV_64FC1);
  frameMeasureGray = Mat::zeros(cv::Size(w,h), CV_64FC1);
  
  if(saveStatisticData)
  {
    tFile.openFile();					// Apertura del file per salvare i tempi di calcolo
    kFile.openFile();					// Apertura del file per salvare i gli stati di kalman durante l'esecuzione
    cFile.openFile();					// Apertura del file per salvare i corners (sono associati ai frame ritagliati che si salvano)
    krFile.openFile();					// Apertura del file per salvare gli stati di kalman durante il recupero (sono associati ai frame ritagliati che si salvano)
  }
    
  // Inizializzazione filtro di Kalman
  stateCovariance << sigmaXAcc, sigmaYAcc;
  measurePosCovariance <<  sigmaXMeasurePos,  sigmaYMeasurePos;
  measureVelCovariance <<  sigmaXMeasureVel,  sigmaYMeasureVel; 
  if(is4StateKalman)
    initialState <<  w/2,0,h/2,0;		// Inizializzo lo stato di kalman a 4 stati
  else
  {
    initialState.resize(6,1);			// Ridefinisco le dimensioni per uniformarmi al 6 stati
    internalState.resize(6);			// Ridefinisco le dimensioni per uniformarmi al 6 stati
    initialState << w/2,0,0, h/2,0,0;		// Inizializzo lo stato di kalman a 6 stati
  }
  u[0] = xAcc; u[1] = yAcc;
  // dichiarazione Filtro di Kalman
  kalmanFilterVideo kf(dt, stateCovariance,measurePosCovariance,measureVelCovariance,initialState);
  kf.initializeFilter();				// Inizializzazione Filtro di Kalman
  kf.initializeTimeMachine(frameToSave, 0);		// Inizializzazione della struttura "timeMachine"
  indexTime = kf.timeMachine.size()+1;
  
  // Inizializzazione della variabile globale "ROI";
  ROI[0] = 0;
  ROI[1] = 1;
  ROI[2] = 0.25; ROI[3] = 0.25;
  ROI[4] = 0.75; ROI[5] = 0.75;
  // Inizializzazione della variabile globale "ROINN"
  ROINN = ROI;
  // Inizializzazione variabili di appoggio OF
  lC = 0;
  bar = barEvaluation(ROI);
  opticalCounter = nFrameOptical + 1;
  opticalCounterStima = nFrameOptical + 1;
  // Frequenze di lavoro
  ros::Rate loopRateWaiting(200);			// Frequenza del ciclo di attesa
  ros::Rate loop_rate(frequency);			// Frequenza del ciclo principale
  // Inizializzazione booleani utili per le fasi della macchina a stati - NON UTILIZZATI
  misura = false;
  optical = false;
  estimation = true;
  actualMeasure = false;
  reset = true;
  trustMeasure = true;
  
  // Sincronizzazione del nodo: il ciclo principale partirà quando tutti i nodi saranno pronti;
  publishReadyStatus();
  waitForTeam();
  // Istante iniziale su cui basarsi per impostare la frequenza del ciclo.
  if(debugDoneWindow)
    namedWindow("Immagine Drone Predizione",CV_WINDOW_NORMAL);
  loop_rate.reset();
  
  
/*----------------------------------------- Ciclo Principale		--------------*/
  while(ros::ok())
  { 
    // tempo di clock inizio ciclo
    gettimeofday(&timeStart, NULL);
    
    // Lettura dei topic
    ros::spinOnce();
    
    /* Se non arriva il primo dato resta in attesa */
    if(!firstMeasure)
    {
      // fraquenza di attesa impostata dal valore "loopRateWaiting"
      // Prendo il tempo di fine ciclo
      gettimeofday(&timeEnd, NULL);
      
      cpu_time_used = (timeEnd.tv_sec - timeStart.tv_sec) * 1e6; 
      cpu_time_used = (cpu_time_used + (timeEnd.tv_usec - timeStart.tv_usec)) * 1e-6; 
      if(saveStatisticData)
      {
	kFile.stato = kf.getState();
	if(kFile.flagNeuralData)
	{
	  kFile.writeDataNeural(currentId);
	  kFile.flagNeuralData = false;
	} 
	else
	  kFile.writeDataNotNeural(currentId);
	
	// Salvataggio dati temporali
	if(tFile.flagNeuralData)
	{
	  tFile.writeDataNeural(currentId, cpu_time_used);
	  tFile.flagNeuralData = false;
	} 
	else
	  tFile.writeDataNotNeural(currentId, cpu_time_used);
      }
      loopRateWaiting.sleep();
      // salto la porzione di codice successiva
      continue;
    }
    
    //cout << "currentId = " << currentId << endl;
    
    /* Reset dello stato se non arriva una misura da un numero pari a "frameToReset" frames*/
    if ((currentId - ROINN[0]) > frameToReset)
    {
      reset = true;
    }
    
    
    
    /* Se la variabile "reset" assume valore "true", quando è disponibile una nuova misura, 
     * lo stato viene resettato alla nuova misura disponibile*/
    if (reset && NewMeasure && ROI[1] != -1)
    {  
      // Se si legge la nuova misura, si segnala con un flag al topic la lettura della nuova misura
      FlagMeasure.data = false;
      // Pubblicazione del messaggio
      pubIsNewMeasure.publish(FlagMeasure);
      // Reset dello stato sfruttando la nuova misura
      kf.resetState(barToState());
      reset = false;
    }
    
    
    
    /* ----------------------Sezione di riaggancio stima al frame corrente----------------------:
       Quando è disponibile una nuova misura, sfruttando il vettore di strutture timeMachine, si effettua la routine
       aggironare la stima al frame corrente. Ciò avviene se è disponibile una nuova misura attendibile dal nodo 
       \NeuralNetworkSim e il frame a cui è riferita la misura è antecedente a quello attuale */
    if (NewMeasure && (currentId > ROI[0]) && ROI[1] != -1)
    {
      FlagMeasure.data = false;
      // Si comunica sul topic l'avvenuta lettura della misura
      pubIsNewMeasure.publish(FlagMeasure);
      // Funzione che identifica l'indice del vettore di strutture a cui è riferita la misura
      indexTime = kf.useTimeMachine(ROI[0]);
      
      /* Se l'indice non è contenuto nella struttura("indexTime == kf.timeMachine.size() + 1"), allora 
        non è possibile effettuare il recupero; Si riporta quindi lo stato del filto di Kalman all'ultima 
        misura disponibile */
      if (indexTime == kf.timeMachine.size() + 1)
      {
	//cout << "Pollini puzza \n";
	misura = false;
	kf.resetState(barToState());
      }
      /* La seguente routine effettua il recupero della stima dal frame identificato dall' indice "indexTime", fino 
       all'id attuale "currentId"*/
      else
      {
	cout << "Frame attuale: " << currentId << "; Misura relativa al frame: " << kf.timeMachine[indexTime].idOld << endl;			//!!!!!!!!!!!!!!!!!!!!!!!!!
	misura = true;
	
	if(saveStatisticData)
	{
	  tFile.flagNeuralData = true;
	  kFile.flagNeuralData = true;
	  kFile.nCountNeural = ROI[0];
	  krFile.nCountNeural = ROI[0];
	}
	opticalStima = false;
	while (kf.timeMachine[indexTime].idOld < currentId)
	{     
	  /* Fase MISURA POSIZIONE: nel frame di misura si calcolano i corners per l'analisi dell'OF su frames successivi, e
	  si imposta il baricentro del drone con il baricentro della ROI.*/
	  
	  if (misura) // MACCHINA A STATI - NON UTILIZZATA
	  {
	    // salvataggio dati ultima misura nella variabile "ROINN"
	    ROINN = ROI;	    
	    if(saveStatisticData)
	    {
	      krFile.stato = kf.getState(); 
	      krFile.writeDataNeural(currentId, kf.timeMachine[indexTime].idOld);
	    }
	    
	    // Il frame è stato salvato in precedenza in formato Gray, viene ritagliato secondo i dati della variabile "ROI"
	    frameCropGray = frameCropperFromROI(ROINN, kf.timeMachine[indexTime].frameOld, idFrameToSave);			//FJ modifica
	    
	    // Si salva il frame ritagliato di misura, per il confronto con i frames successivi per il calcolo dell'OF.
	    frameCropGray.copyTo(frameMeasureGray);
	    // Aggiorno la variabile distanceCorners
	    calDistanceBetweenCorners(ROINN);
	    // Individuazione dei corners di cui tenere traccia (Metodo LK)
	    goodFeaturesToTrack (frameCropGray,corners, nCorners, thresholdCorners, distanceCorners, Mat(), 3, 3, 0, 0.04);		//0.15 - 2 - Mat - 3 - 3 - 0 - 0.04
	    if(saveStatisticData) 
	      cFile.writeDataNeural(idFrameToSave++, corners, currentId);
	    // Numero di corners individuati nel frame di misura
	    lC = corners.size();
	    /* Si pone a "true" il flag di individuazione del singolo corner*/
	    for (int i = 0; i < lC; i++)
	    {
	      newstatus[i] = 1;
	    }
	    // Aggiornamento variabile "corners"
	    //	corners = cornersMeasure;
	    // Inizializzo il contatore del numero di frame su cui applicare OF
	    
	    // Calcolo del baricentro secondo i dati della detection
	    bar = barEvaluation(ROI);
	    if(saveStatisticData)
	      krFile.stFile << "BAR NEURAL x= " << bar[0] << "\ty=" << bar[1] << endl;
	    // Correzione dello stato di Kalman, sfruttando la misura di posizione del baricentro
	    internalState = kf.getState();
	    bar = kf.kalmanCorrectionPosition(evaluateAcceleration(internalState),bar);
	    // Incremento contatore dei frame da recuperare
	    indexTime++;
	    /* Uscita dal ciclo se sono stati analizzati tutti i frames da recuperare*/
	    if ( indexTime == kf.timeMachine.size())
	    {
	      actualMeasure = true;
	      break;
	    }
	    // Fine della fase di misura
	    misura = false;
	    // Inizio del calcolo OF
	    optical = true;
	    /* Se non sono stati individuati corners su cui effettuare OF discreto nel frame di misura o
	      o non si vuole effettuare il calcolo dell' OF ("nFrameOptical = 0"), si salta la fase di misura della velocità
	      ("optical = false") e si passa alla fase di stima.*/
	    if (lC == 0)		//lC == 0
	    {
	      optical = false;
	      estimation = false;
	      break;
	    }
	  }
	  
	  
	  /*Fase MISURA OPTICAL: a partire dal frame successivo alla misura si calcola la velocità di spostamento del baricentro 
	    del drone, sfruttando le funzioni dell'OF discreto.*/
	  if (optical)  // MACCHINA A STATI - NON UTILIZZATA
	  {
	    if(saveStatisticData)
	    {
	      krFile.stato = kf.getState();
	      krFile.writeDataNeural(currentId, kf.timeMachine[indexTime].idOld);
	    }
	    
	    // Ritaglio del frame su cui effettuare OF. I dati della ROI !!sono sempre quelli relativi al frame di misura della posizione!!
	    frameCropGray = frameCropperFromROI(ROINN, kf.timeMachine[indexTime].frameOld, idFrameToSave); 					//FJ modifica
	    
	    // Salvataggio dei corners individuati al frame precedente, per calcolo velocità
	    cornersold = corners;
	    // Individuazione degli stessi corners individuati nel frame di misura, nel frame attuale
	    corners = newCorners(frameMeasureGray, frameCropGray, corners);										//ERRORE!
	    if(saveStatisticData) 
	      cFile.writeDataNeural(idFrameToSave++, corners, currentId);
	    
	    // Calcolo della velocità di spostamento del baricentro rispetto al frame precedente
	    vel = newVel(cornersold, corners);		//vel = [vx, vy]
	    
	    //Sposto il baricentro della ROI dinamica ROINN e, quindi, anche i corner ad essa associati						//FJ INIT MODIFICA
	    ROINN[2] += vel[1]*dt/h;
	    ROINN[4] += vel[1]*dt/h;
	    
	    ROINN[3] += vel[0]*dt/w;
	    ROINN[5] += vel[0]*dt/w;
	    
	    frameCropGray = frameCropperFromROI(ROINN, kf.timeMachine[indexTime].frameOld, idFrameToSave);			//FJ modifica
	    
	    // Si salva il frame ritagliato di misura aggiornata, per il confronto con il frame successivo per il calcolo dell'OF.
	    frameCropGray.copyTo(frameMeasureGray);
	    // Individuazione dei corners di cui tenere traccia (Metodo LK)
	    goodFeaturesToTrack (frameCropGray,corners, nCorners, thresholdCorners, distanceCorners, Mat(), 3, 3, 0, 0.04);
	    // Numero di corners individuati nel frame di misura
	    if(saveStatisticData)
	      cFile.writeDataNeural(idFrameToSave++, corners, currentId);
	    lC = corners.size();
	    /* Si pone a "true" il flag di individuazione del singolo corner*/
	    for (int i = 0; i < lC; i++)
	    {
	      newstatus[i] = 1;
	    }
	    
	    indexTime++;
	    // Correzione dello stato di Kalman, sfruttando la misura di velocità del baricentro
	    internalState = kf.getState();
	    bar = kf.kalmanCorrectionVelocity(evaluateAcceleration(internalState), vel);
	    // Uscita dal ciclo se sono stati analizzati tutti i frames da recuperare
	    if ( indexTime == kf.timeMachine.size())
	    {
	      actualMeasure = true;
	      break;
	    }
	    // Se il numero di frame su cui è stato effettuato il calcolo dell' OF è >= a "nFrameOptical", si passa alla fase di stima
	    if (lC == 0)		//lC == 0
	    {
	      optical = false;
	      estimation = false;
	      break;
	    }
	  }
	  
	  
	  /* Fase STIMA: si ricalcolano con il filtro di Kalman gli stati precedenti del sistema fino al frame attuale. 
	    Ciò avviene nei frames compresi tra quello di misura e il frame identificato con "currentId"*/
	  if (estimation)  // MACCHINA A STATI - NON UTILIZZATA
	  {
	    if(saveStatisticData)
	    {
	      krFile.stato = kf.getState();
	      krFile.writeDataNeural(currentId, kf.timeMachine[indexTime].idOld);
	    }
	    
	    // Stima baricentro
	    internalState = kf.getState();
	    bar = kf.kalmanEstimation(evaluateAcceleration(internalState));
	    // Incremento contatore dei frame da recuperare
	    indexTime++;
	    // Uscita dal ciclo se sono stati analizzati tutti i frames da recuperare
	    if ( indexTime == kf.timeMachine.size())
	    {
	      actualMeasure = true;
	      break;
	    }
	  }
	}
	// Flag che indica la fine della fase di stima
	estimation = false;
	// Flag che assume valore "true" se la fase di recupero è finita
	actualMeasure = true;
      }
    }
    
    
    
    /* Routine che viene richiamata nel caso si voglia simulare il comportamento del sistema in cui 
     * non è presente ritardo dovuto alla computazione della misura ("currentId == ROI[0]") */
    if (currentId == ROI[0] && NewMeasure)
    {
 //     cout << "Misura relativa al frame: " << ROI[0] <<"; Id del frame: " << currentId << endl; 						//!!!!!!!!!!!!!!!!!!
      ROINN = ROI;			
      FlagMeasure.data = false;
      pubIsNewMeasure.publish(FlagMeasure);
      bar = barEvaluation(ROI);
      internalState = kf.getState();
      bar = kf.kalmanCorrectionPosition(evaluateAcceleration(internalState), bar);
      actualMeasure = true;
      opticalStima = false;
      if(saveStatisticData)
	tFile.flagNeuralData = true;	// Inserito per scrivere nel file di testo i calcoli del calcolo temporale
    }
    
    
    
    /*//----------------------Sezione di stima sul frame attuale----------------------//
     * Routine che viene eseguita dopo il riaggancio dello stato del sistema al frame attuale, finchè non è disponibile
     * una nuova misura:
     * Fase MISURA OPTICAL: dopo il riaggancio al frame corrente, si calcola la velocità di spostamento del baricentro 
     * del drone, sfruttando le funzioni dell'OF discreto.*/
    if (opticalStima)
    {
      // Ritaglio del frame attuale secondo ROINN
      frameCrop = frameCropperFromBar(ROINN, frame, barMeasure, idFrameToSave);								//FJ modifica
      
      // Da RGB a Gray
      cvtColor(frameCrop, frameCropGray, COLOR_BGR2GRAY);
      
      // Salvataggio dei corners individuati al frame precedente, per calcolo velocità
      cornersold = corners;

      // Individuazione degli stessi corners individuati nel frame di misura, nel frame attuale
      corners = newCorners(frameMeasureGray, frameCropGray, corners);
      if(saveStatisticData)
	cFile.writeDataNotNeural(idFrameToSave++, corners, currentId);

      // Calcolo della velocità di spostamento del baricentro rispetto al frame precedente
      vel = newVel(cornersold, corners);
      // Incremento del contatore dei frame su è stato calcolato OF
      
      // Correzione dello stato di Kalman, sfruttando la misura di velocità del baricentro
      internalState = kf.getState(); 
      bar = kf.kalmanCorrectionVelocity(evaluateAcceleration(internalState), vel);
      
      frameCrop = frameCropperFromBar(ROINN, frame, bar, idFrameToSave);									//FJ modifica
      // Da RGB a Gray
      cvtColor(frameCrop, frameCropGray, COLOR_BGR2GRAY);
      // Si salva il frame ritagliato di misura, per il confronto con i frames successivi per il calcolo dell'OF.
      frameCropGray.copyTo(frameMeasureGray);
      // Individuazione dei corners di cui tenere traccia (Metodo LK)
      goodFeaturesToTrack (frameCropGray,corners, nCorners, thresholdCorners, distanceCorners, Mat(), 3, 3, 0, 0.04);
      // Numero di corners individuati nel frame di misura
      lC = corners.size();
      if(saveStatisticData)
	cFile.writeDataNotNeural(idFrameToSave++, corners, currentId);
      //Si pone a "true" il flag di individuazione del singolo corner
      for (int i = 0; i < lC; i++)
      {
	newstatus[i] = 1;
      }
      barMeasure = bar;
      
      // Debug Drone
      if(debugDoneWindow)
      {
	imshow("Immagine Drone Predizione", frameCropGray);
	waitKey(1);
      }
      
      // Controllo se ci sono corner su cui applicare l'OF
      if(lC == 0)		// lC == 0
      {
	opticalStima = false;
	trustMeasure = false;
      }
    }
    
    
    
    /*Fase MISURA POSIZIONE: nel frame attuale si calcolano i corners per l'analisi dell'OF su frames successivi, 
     utilizzando la ROI ricavata dal calcolo del baricentro effettuata nella fase di recupero. La ROI risulta incrementata
     di un fattore "aplha" per tenere conto dell'incertezza di tale stima*/
    if (actualMeasure)
    {
      // Se arriva una nuova misura, "trustMeasure" viene impostato a "true"
      trustMeasure = true;
      // Ritaglio del frame attuale secondo il baricentro stimato nella fase di recupero
      frameCrop = frameCropperFromBar(ROINN, frame, bar, idFrameToSave);									//FJ modifica
      
      // Da RGB a Gray
      cvtColor(frameCrop, frameCropGray, COLOR_BGR2GRAY);
      // Si salva il frame ritagliato di misura, per il confronto con i frames successivi per il calcolo dell'OF.
      frameCropGray.copyTo(frameMeasureGray);
      // Individuazione dei corners di cui tenere traccia (Metodo LK)
      goodFeaturesToTrack (frameCropGray,corners, nCorners, thresholdCorners, distanceCorners, Mat(), 3, 3, 0, 0.04);
      // Numero di corners individuati nel frame di misura
      lC = corners.size();
      if(saveStatisticData)
	cFile.writeDataNotNeural(idFrameToSave++, corners, currentId);
      //Si pone a "true" il flag di individuazione del singolo corner
      for (int i = 0; i < lC; i++)
      {
	newstatus[i] = 1;
      }
      actualMeasure = false;
      opticalStima = true;
      
      /* barMeasure assume il valore del baricentro calcolato alla fine della fase di recupero. Tale variabile 
      serve a mantenere coerenza nel ritaglio della ROI*/
      barMeasure = bar;
      if (lC == 0)		//lC == 0
      {
	//			opticalCounterStima = nFrameOptical + 1;
	opticalStima = false;
	trustMeasure = false;
      }
    }
    
    
    /* Avvio la fase di frenata del filtro di Kalman, causato da un alto numero di frame mancati dalla rete neurale*/
    if ((currentId - ROINN[0]) > frameToBrake)
    {
     trustMeasure = false;
    }
    
    
    /* Se non ricevo alcuna misura dalla rete neurale e l'OF non riporta alcun dato, stimo senza correzioni
       Inserito anche il controllo se il numero di mancati frame dalla rete neurale supera il limite acconsentito */
    if (!NewMeasure &&  !opticalStima)
    {
      // "True" se non ricevo misura da nFramesBrek
      if (!trustMeasure)
      {
	/* Azione di frenatura nel caso in cui la rete fallisce */
	internalState = kf.getState();
	if(is4StateKalman)
	{
	  u[0] = - kBrake * internalState [1] / dt;
	  u[1] = - kBrake * internalState [3] / dt;
	}
	else
	{
	  u[0] = - kBrake_vel * internalState [1] - kBrake_acc * internalState [2];
	  u[1] = - kBrake_vel* internalState [4] - kBrake_acc * internalState [5];
	}
      }
      else
      {
	//	cout << "stima normale\n";
	internalState = kf.getState();
	u = evaluateAcceleration(internalState);
      }
      /* Stima */
      bar = kf.kalmanEstimation(u);
      u[0] = xAcc;
      u[1] = yAcc;
    }
    
    
    // Conversione frame attuale in Gray
    cvtColor(frame, frameToSave, COLOR_BGR2GRAY);
    // Salvataggio del frame attuale(Gray) e del corrispettivo id a cui si riferisce nella struttura "timeMachine"
    kf.insertParams(frameToSave, currentId);
    // Pulizia buffer prima dell'invio del nuovo messaggio
    barOut.data.clear();
    RoiOut.data.clear();
    // Inserimento del dato nel messaggio
    RoiOut.data.insert(RoiOut.data.end(), ROINN.begin(), ROINN.end());
    barOut.data.insert(barOut.data.end(), bar.begin(), bar.end());
    // Scrittura sul topic
    bar_pub.publish(barOut);
    roi_pub.publish(RoiOut);
    
    
    // Prendo il tempo di fine ciclo
    gettimeofday(&timeEnd, NULL);
    
    cpu_time_used = (timeEnd.tv_sec - timeStart.tv_sec) * 1e6; 
    cpu_time_used = (cpu_time_used + (timeEnd.tv_usec - timeStart.tv_usec)) * 1e-6; 
    // Attesa fino al raggiungimento del periodo del nodo
    loop_rate.sleep();
    
    if(saveStatisticData)
    {
      kFile.stato = kf.getState();
      if(kFile.flagNeuralData)
      {
	kFile.writeDataNeural(currentId);
	kFile.flagNeuralData = false;
      } 
      else
	kFile.writeDataNotNeural(currentId);
      
      // Salvataggio dati temporali
      if(tFile.flagNeuralData)
      {
	tFile.writeDataNeural(currentId, cpu_time_used);
	tFile.flagNeuralData = false;
      } 
      else
	tFile.writeDataNotNeural(currentId, cpu_time_used);
    }
      
  }
  
  if(saveStatisticData)
  {
    tFile.closeFile();		// Chiusura del file
    kFile.closeFile();		// Chiusura del file
    cFile.closeFile();		// Chiusura del file
    krFile.closeFile();		// Chiusura del file
  }
  return 0;
}
