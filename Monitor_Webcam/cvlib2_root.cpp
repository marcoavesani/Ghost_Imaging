#include <iostream>
#include <cv.h>
#include <highgui.h>

#include "TH2D.h"
#include "TFile.h"
using namespace cv;
using namespace std;

//Flag globale per la gestione dei colori nell'elaborazione dell'immagine
int flag=0; // Global Flag Variable
//Funzioni di callback per i bottoni. Quando seleziono un bottone questo esegue questa funzione
void callbackButton(int state,void* userdata)
{
    flag=0;
}

void callbackButton2(int state,void* userdata)
{
    flag=1;
}

void callbackButton3(int state,void* userdata)
{
    flag=2;
}
//Prototipo della funzione transform. Questa si preoccupa di gestire il colore in base cosa viene passato al flag 
void transform(cv::Mat &input,cv::Mat &output);

void save_root(cv::Mat &mat,string filename,double w,double h);
int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
    cout  << "Cannot open the video file" << endl;
        return -1;
    }

   //double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   //double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the  video
    
   double dWidth=1280;
   double dHeight =720;
   /*
   double dWidth=640;
   double dHeight =480;
   */
   
   cap.set(CV_CAP_PROP_FRAME_WIDTH, dWidth);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT, dHeight);
   cap.set(CV_CAP_PROP_FPS, 30);
   //cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.0 ); 
   //cap.set(CV_CAP_PROP_GAIN, 0.0 ); 
   //cap.set( CV_CAP_PROP_EXPOSURE, 0.0 ); 
   system("v4l2-ctl -c exposure_auto=1 -d /dev/video0");
   system("v4l2-ctl -c exposure_absolute=5 -d /dev/video0");
   cout << "Frame size : " << dWidth << " x " << dHeight << endl;
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    //Create trackbars
     int iSliderValue1 = 0;
    createTrackbar("Gain", "MyVideo", &iSliderValue1, 100.);
    int iSliderValue2 = 0;
    createTrackbar("Exposure", "MyVideo", &iSliderValue2, 100.);
    /*
    //Create Buttons 
     cv::createButton("RGB",callbackButton,NULL,CV_RADIOBOX,1); // Button 1 
     cv::createButton("GRAYSCALE",callbackButton2,NULL,CV_RADIOBOX,0); // Button 2
     cv::createButton("HSV",callbackButton3,NULL,CV_RADIOBOX,0); // Button 3
    // cv::createButton("HSV",callbackButton3,NULL,CV_RADIOBOX,0); // Button 3
     */
     //Creo matrice temporanea x salvare frame della webcam
     Mat frame;
     string nome_out;
     int angolo=0;
     int passo=5;
     while (1){
    //Setto valori di esposizione e gain attraverso i trackbar
    // cap.set(CV_CAP_PROP_GAIN, (iSliderValue1)/100.); 
    // cap.set( CV_CAP_PROP_EXPOSURE, (iSliderValue2)/100.); 
   //Piglio immagine dalla webcam e la sbatto in frame
    cap.retrieve(frame);
     // cap >> frame; 
    bool bSuccess = cap.read(frame); // read a new frame from video
       if (!bSuccess) //if not success, break loop
        {
         cout << "Cannot read a frame from video file" << endl;
            break;
        }
	//Applico trasformazione dei colori
	//transform(frame, frame);
        imshow("MyVideo", frame); //show the frame in "MyVideo" window
   
	int pressed =waitKey(10);
       
	if ( pressed == 27) 
        //if (waitKey(10) == 'a') 
	{
        cout << "esc key is pressed by user" << endl;
          break; 
        }
        else if(pressed== 'a'){
	//Modalità scatto singolo
	cout<<"Dimmi il nome del file"<<endl;
	cin>>nome_out;
	//cv::FileStorage storage("test.yml", cv::FileStorage::WRITE);
	//storage << "img" << frame;
	//storage.release();  
	string root=nome_out+".root";
	string bmp=nome_out+".bmp";
	//cout<<root<<"\t"<<bmp<<endl;
	save_root(frame,root.c_str(),dWidth,dHeight);
	imwrite(bmp.c_str(),frame);
	}
	else if(pressed== 'x'){
	//Modalità raffica
	if(angolo==0){
	  cout<<"Dimmi l'angolo da cui partire"<<endl;
	cin>>angolo;
	}
	cout<<"Siamo all'angolo "<<angolo <<endl; 
	ostringstream convert;
	convert << angolo;
	
	string root=convert.str()+".root";
	string bmp=convert.str()+".bmp";
	//cout<<root<<"\t"<<bmp<<endl;
	save_root(frame,root.c_str(),dWidth,dHeight);
	imwrite(bmp.c_str(),frame);
	angolo=angolo+passo;
	  
	}  
    Point3_<uchar>* p = frame.ptr<Point3_<uchar> >(100,100);
    cout<<int(p->x)<<"\t"<<int(p->y)<<"\t"<<int(p->z)<<"\t"<<endl;
      
      
    }
    return 0;

}
 void transform(cv::Mat &input,cv::Mat &output)
{
    switch(flag) // Depending on the value of flag variable we have our output image tranformed
        {
            case(0):
                input.copyTo(output); // RGB
                break;
            case(1):
                cv::cvtColor(input,output,CV_BGR2GRAY); // GRAYSCALE
                break;
            case(2):
                cv::cvtColor(input,output,CV_BGR2HSV); // HSV
                break;  
        }   
}

 void save_root(cv::Mat &mat,string filename,double w,double h){
  TFile * file=new TFile( filename.c_str(),"RECREATE");
  TH2D * blue= new TH2D("blue","blue",w,0,w,h,0,h);
  TH2D * green= new TH2D("green","green",w,0,w,h,0,h);
  TH2D * red= new TH2D("red","red",w,0,w,h,0,h);
  for(int i=0;i<w;i++){
    for(int j=0;j<h;j++){
  Point3_<uchar>* p = mat.ptr<Point3_<uchar> >(j,i);
  red->SetBinContent(w-i,h-j,double(p->z));
  green->SetBinContent(w-i,h-j,double(p->y));
  blue->SetBinContent(w-i,h-j,double(p->x));
    }
  }

red->Write();
green->Write();
blue->Write();
file->Close();
  
 return ; 
}
