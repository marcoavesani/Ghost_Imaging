#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <iostream>
#include <cv.h>
#include "opencv2/opencv.hpp"
#include <highgui.h>
//#include <thread>
#include <boost/thread.hpp> //Boost multithreading library

#include "TH2D.h"          /* ROOT INCLUDE */
#include "TFile.h"
#include "TApplication.h"
#include "TCanvas.h"
#include "TSystem.h"
#include "TStyle.h"
#include "TVectorD.h" 
/* NAMESPACES */
using namespace std;
using namespace cv;

/*PROTOTIPI */
int serialport_init      (const char* serialport, int baud);
//int serialport_read_until(int fd, char* buf, char until);
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout);
int serialport_write     (int fd, const char* str);
void serialport_read     (int fd, char*       buf, char until);
void save_root           (cv::Mat &mat,std::string filename,double w,double h,double foto,int loop ,bool nuovo);
void plot_root();
void captureFunc(Mat *frame, VideoCapture *capture,double dWidth,double dHeight);
double calibrazione (double x);
/*GLOBALI*/
TApplication theApp("A",0,0); 			// Costruttore Classe TApplication per poter plottare i dati, non è legata al argv o argc perchè viene eseguita
                                                // su un thread separato e non nel main
string nomefile="ghost_image.root"; 		//Nome del file in output
bool is_opened=false;				//Flag globale per una gestione grezza ma funzionale della sincronizzazione tra i thread

//Main
int main(int argc, char **argv) 
{	gStyle-> SetCanvasPreferGL(kTRUE);              //Se ROOT è stato compilato con supporto OpenGL, le utilizzo per avere migliori performance
	gStyle->SetNumberContours(255);                 //Opzioni grafiche varie
	gStyle->SetPalette(1);
        //Inizializzazioni variabili
    	int fd = 0; int loop=0; int rc;
    	const char* serialport = "/dev/ttyACM0"; 			//Apertura seriale	
    	char buf[256];							//Creazione buffer per la lettura
	//char buf[1024];
	const char* fotodiodo = "F";					//Selezione comandi per attivazione eventi su arduino
	const char* motore = "M";
	//Inizializzazione seriale
	fd = serialport_init(serialport, 9600);				//apri la porta
	sleep(1);							//Dai tempo all'arduino di resettarsi e ripartire
	
	//Inizializzazione webcam
	double dWidth=1280;
        double dHeight =720;
	//double dWidth=640;
        //double dHeight =480;
        
        cv::VideoCapture cap(0); 					//Apri la sorgente di cattura numero 0 (nel caso ci siano più sorgenti di cattura nel sistema)
	if (!cap.isOpened())  						// Se non ce la fai, dimmi che c'è qualcosa che non va e torna a casa
	  {
	    std::cout  << "Cannot open the video file" << std::endl;
	    return -1;
	  }
	
	//Settaggio proprietà webcam
	
	//double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 		//Fatti dare direttamente dal driver la risoluzione
	//double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); /
	//cout<<"WITH: "<<  dWidth<<"Height: "<<dHeight<<endl;
	
        cap.set(CV_CAP_PROP_FRAME_WIDTH, dWidth);			//Settaggio manuale risoluzione webcam
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, dHeight);
	cap.set(CV_CAP_PROP_FPS, 30);					//Settaggio manuale FPS(Frame per Second)
	//cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.0 ); 			//Settaggio tramite opencv della proprietà della webcam ( attualmente 
	//cap.set(CV_CAP_PROP_GAIN, 0.0 ); 				//buggato perchè usa backend v4l invece di v4l2, soluzione cambiare sorgente di
	//cap.set( CV_CAP_PROP_EXPOSURE, 0.0 ); 			//opencv in highgui) inibito, si usa v4l2-ctl
	
	system("v4l2-ctl -c exposure_auto=1 -d /dev/video0");		//Uso v4l2-ctl per settare al minimo il gain
	system("v4l2-ctl -c exposure_absolute=5 -d /dev/video0");       // absolutee=5 è il minimo
	namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);
	
        boost::mutex mut;
        cv::Mat frame,frame_temp;                                                  //Creazione di un oggetto mat( in pratica un tensore di rk 3 o 4 con lo scan bidimnesionale x i
        
        setlocale(LC_NUMERIC,"C");                                      //3 canali RGB + 1 Somma) per lo storage temporaneo in RAM
        
        cap>>frame;
        cap>>frame_temp;

	//boost::thread thread_1 = boost::thread(plot_root);
	boost::thread thread_1 (&plot_root);				//Lancio un thread separato asincrono che si occupa di plottare ogni tot secondi i dati
	boost::thread captureThread(captureFunc, &frame_temp, &cap,dWidth,dHeight);
        
        while(1){	
		cout<<"Loop: "<<loop<<endl;
		/*
		int pressed =waitKey(10);				//Parte di codice che si occupa dell'uscita dal programma
		//if ( pressed == 27) 
		if (waitKey(10) == 'a') 
		{
		cout << "esc key is pressed by user" << endl;
		break; 
		}
		*/
		mut.lock();
                frame=frame_temp.clone();
		mut.unlock();
		rc = serialport_write(fd, fotodiodo);			//Mando all'arduino attraverso la seriale il comando di leggere la tensione sul fotodiodo
		sleep(0.1);
		serialport_read(fd, buf, '\n');
		sleep(0.5);
		rc = serialport_write(fd, motore);			//Mando il comando all'arduino di girare il motore
		
		std::cout << buf;					//Stampo a schermo quello che esce (DEBUG)
		double volt_foto = atof(buf);				//Converto il numero che però è codificato in char nel vero numero e lo sbatto in un double
		std::cout << "atof " << volt_foto << std::endl;
		//plot_root(nomefile.c_str());
		cout<< gSystem->AccessPathName(nomefile.c_str())<<endl;         //Metodo di ROOT per vedere se esiste un rootfile con il nome scelto
		if(gSystem->AccessPathName(nomefile.c_str()) && loop>1){	// Se il file esiste, scarto il primo frame ( che può avere artefatti) e dal secondo in poi salvo  
		save_root(frame,nomefile.c_str(),dWidth,dHeight,volt_foto,loop,true);//Prendo il frame e il valore di fotodiodo e li aggiungo all'istogramma che avevo salvato su file
		 }
		else{                                                           //Altrimenti se il file non esiste, scarto sempre il primo frame e creo un nuovo file e un nuovo hist
		  if(loop>1){
		  save_root(frame,nomefile.c_str(),dWidth,dHeight,volt_foto,loop,false);
		  }
		 }
		//cout<<"DEBUG"<<endl;
								//Speto 3 secondi che nn si sa mai quanto ci impega a scrivere su file
		memset( buf, 0, sizeof(buf) );				//Cancello il contenuto di buf per il nuovo ciclo
		//Qua finisce e ricomincia il loop
		
                imshow("MyVideo", frame);
                int pressed =waitKey(10);
       
                if ( pressed == 27) 
                //if (waitKey(10) == 'a') 
                {
                 cout << "esc key is pressed by user" << endl;
                 break; 
                } 
                sleep(1.5);
                loop++;
	}
		
		destroyWindow("MyVideo");
		thread_1.interrupt();					//Quando esco dal ciclo interrompo in maniera safe il thread creato
		thread_1.join();					//Join è più di sicurezza che altro.. Non c'è nessun merge da fare
		captureThread.interrupt();
                captureThread.join();
                cap.release();
                return 0; 
}



int serialport_init(const char* serialport, int baud)			//Da qui in poi fede
{
    struct termios toptions;
    int fd;
	
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

	speed_t brate = baud;
   	cfsetispeed(&toptions, brate);
    	cfsetospeed(&toptions, brate);

    // 8N1 robe
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control altre robe
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1; }

    return fd;
}

void serialport_read(int fd, char* buf, char until)
{
    char* b =  new char[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);
        //std::cout << n << std::endl;
       // std::cout << "i: " << i << "      buf: " << b << std::endl; // read a char at a time
        if( n==-1){ std::cout << "Non riesco a leggere la porta!"<< std::endl;  return; }   // couldn't read
        if( n==0 ) {  // wait 1 msec try again
            continue;
        }
        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i <= 256);

	buf[i] = 0;  // null terminate the string
    return;
}


int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }
#ifdef SERIALPORTDEBUG  
        printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}


/*
int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
	if( n==0 ) {
	usleep( 10 * 1000 );  // wait 10 msec try again
	continue; }
        buf[i] = b[0]; i++;
   } while( b[0] != until );
	
	buf[i] = 0;  // null terminate the string
	return 0;
}
*/
/*
int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 10 * 1000 ); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until );

    buf[i] = 0;  // null terminate the string
    return 0;
}
*/
int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) 
        return -1;
    return 0;
}


void plot_root(){  					//Da qui smette la fede
  try							//Visto che con i thread le cose possono incasinarsi per bene uso il sistema di exception
  {							// qui ci va il codice da provare a eseguire
   
    TCanvas* c1 = new TCanvas("c1","Canvas 1"); 	//Creo un canvas come pointer che rimarrà allocato x tutto il tempo in modo da non sprecare memoria a ricrearlo a ogni ciclo
    TH2D * red;					// il puntatore dell'istogramma da visualizzare viene definito fuori dal ciclo x essere sempre accessibile
       
   while(1){						// Il ciclo gira sempre, l'interruption point si occupa di fare un break
    boost::this_thread::interruption_point();    
    if(!is_opened){
     if(!gSystem->AccessPathName(nomefile.c_str())){ 	//Controllo che il file esista
     TFile file(nomefile.c_str(),"READ" );		//Se è così lo apro in sola lettura	
      red=(TH2D*)file.Get("red");			//Prendo l'istogramma che mi interessa
      red->SetDirectory(0);				//Lo carico in memoria invece che lasciarlo attaccato al file
      file.Close();					//Chiudo il file che meno lo lascio aperto meglio è
      red->Draw("colz");				//Plotto
      c1->Update();					//Aggiorno il canvas
      //sleep(3);						//Aspetto tre secondi      
      boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
      delete red;					//Dealloco la memoria per evitare memory leak
      boost::this_thread::interruption_point();		//Metto un'interruption_point x sapere dove fermare il thread
      
      cout<<"Chiudo e riapro"<<endl;			//DEBUG
      //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
     }
     
     else{						//Se il file non è aperto aspetto 3 secondi
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));}
     }
     else{
       boost::this_thread::sleep(boost::posix_time::milliseconds(100));//Altrimenti se ho avuto la fortuna di beccarlo mentre l'altro thread stava scrivendo aspetto solo 100 millisec
     }
     
  }							//Chiudo il Loop  
  c1->Close();
   
  }							//Chiudo il TRY
  
  catch(boost::thread_interrupted&){			//Se si verifica un exception la catturo qui
    theApp.Terminate();
    cout << "Thread is stopped" << endl;		//Stampo un messaggio d'errore
            return;					//Chiudo tutto il programma
  }
  return;
  
}


 void save_root(cv::Mat &mat,string filename,double w,double h,double foto,int loop,bool nuovo){
  
  //imwrite("outFdileName.bmp",mat);
  //TFile * file;						//Alloco i puntatori ai vari oggetti fuori dagli if in modo
  TVectorD misure(1);
  misure[0]=loop;
  TH2D * blue;							// che siano sempre accessibili
  TH2D * green;
  TH2D * red;
  if (nuovo){							//Se viene passato il flag nuovo
    blue=new TH2D("blue","blue",w,0,w,h,0,h);			//Creo dei nuovi istogrammi bidimensionali vuoti, uno x canale
    green= new TH2D("green","green",w,0,w,h,0,h);
    red= new TH2D("red","red",w,0,w,h,0,h);
    
    for(int i=0;i<w;i++){					//Faccio un loop su tutto riempiendo gli istogrammi con i valori presi dall'oggetto mat di opencv
      for(int j=0;j<h;j++){
	Point3_<uchar>* p = mat.ptr<Point3_<uchar> >(j,i); 	//Oggetto x accedere al tensore, fissato il punto del tensore (y,x) mi da un vettore
	red->SetBinContent(w-i,h-j,  calibrazione(double(p->z))*foto);
	green->SetBinContent(w-i,h-j,calibrazione(double(p->y))*foto);
	blue->SetBinContent(w-i,h-j, calibrazione(double(p->x))*foto);
	}
    }
    
    is_opened=true;
    TFile file ( filename.c_str(),"RECREATE");			//Apro il nuovo file e se ce n'è uno lo sovvrascrivo
    red->Write();						//Scrivo tutto su file
    green->Write();
    blue->Write();
    misure.Write("misure");
    file.Close();						//Chiudo il file
    is_opened=false;
    
  }
  else{							//Altrimenti apro il file già esistente in lettura/scrittura senza cancellare
    is_opened=true;
    TFile file ( filename.c_str(),"UPDATE");		// e prendo gli istogrammi in memoria
    blue= (TH2D*) file.Get("blue");
    green=(TH2D*)file.Get("green");
    red=  (TH2D*)  file.Get("red");
    
    for(int i=0;i<w;i++){				// Leggo i valori che ci sono già nei singoli canali
      for(int j=0;j<h;j++){				// punto x punto e li salvo
	double red_old  =red->  GetBinContent(w-i,h-j);	
	double green_old=green->GetBinContent(w-i,h-j);
	double blue_old =blue-> GetBinContent(w-i,h-j);
  
	Point3_<uchar>* p = mat.ptr<Point3_<uchar> >(j,i);          //Aggiorno i valori esistenti  
	red->  SetBinContent(w-i,h-j, (red_old   +calibrazione(double(p->z))*foto));
	green->SetBinContent(w-i,h-j, (green_old +calibrazione(double(p->y))*foto));
	blue-> SetBinContent(w-i,h-j, (blue_old  +calibrazione(double(p->x))*foto));
	} 
      }
    
    red->Write(red->GetName(),    TObject::kOverwrite);			//Salvo i file sovvrascrivendo quelli che già ci sono
    green->Write(green->GetName(),TObject::kOverwrite);
    blue->Write(blue->GetName(),  TObject::kOverwrite);
    misure.Write("misure",TObject::kOverwrite);
    file.Close();//Chiudo il file e vado a casa
    is_opened=false;
  }
  
  						
  //delete file;
  //delete green;							//Dealloco la memoria
  //delete red;
  //delete blue;
 
  return ; 
}
// Code for capture thread
void captureFunc(Mat *frame, VideoCapture *capture,double dWidth,double dHeight){
      try{  
        capture->set(CV_CAP_PROP_FRAME_WIDTH, dWidth);                      //Settaggio manuale risoluzione webcam
        capture->set(CV_CAP_PROP_FRAME_HEIGHT, dHeight);
  //loop infinitely
  for(;;){
                //capture from webcame to Mat frame
                (*capture) >> (*frame);
                boost::this_thread::interruption_point();
   
  }
      }
  catch(boost::thread_interrupted&){                      //Se si verifica un exception la catturo qui
    
    cout << "Thread2 is stopped" << endl;                //Stampo un messaggio d'errore
            return;             
       
      }
}

double calibrazione (double x){
 //double temp= pow(x,1);
 return x;
}