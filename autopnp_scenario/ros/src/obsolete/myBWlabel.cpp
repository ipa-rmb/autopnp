#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ctime>
#include <vector>
#include <map>
using namespace std;

int myLabel(IplImage* I,vector<CvPoint>* regions);

int main(){
	
	IplImage* I = cvLoadImage("boxes2.jpg",0);
	IplImage* labelled_img=cvCreateImage(cvSize(I->width,I->height),8,3);
	
	double diff;
	clock_t start;	
	
	vector<CvPoint>* R=new vector<CvPoint>[20];
	int numOfLabels;
	uchar* Lpixel;
	
	cvThreshold(I,I,127,255,CV_THRESH_BINARY);	
	
	start = std::clock();									//TIMING START
	
	numOfLabels=myLabel(I,R);
	
	diff = ( clock() - start ) / (double)CLOCKS_PER_SEC;	//STOP TIMING
	printf("TIME: %f\n",diff);
	
	//Example of use: randomly color on each region
	int i,l,h,w,r,g,b;
	Lpixel=(uchar*)labelled_img->imageData;
	for(l=0;l<numOfLabels;l++){
		r=rand();
		g=rand();
		b=rand();
		for(i=0;i<R[l].size();i++){
			h=R[l].at(i).x;
			w=R[l].at(i).y;
			Lpixel[h*labelled_img->widthStep +w*3 +0]=b;
			Lpixel[h*labelled_img->widthStep +w*3 +1]=g;
			Lpixel[h*labelled_img->widthStep +w*3 +2]=r;
		}
	}	

	cvNamedWindow( "Labels", 1 );
	cvShowImage( "Labels", labelled_img );

	cvNamedWindow( "bin img", 1 );
	cvShowImage( "bin img", I );
	cvWaitKey(0);

}

int myLabel(IplImage* I,vector<CvPoint>* regions){
	
	int height = I->height;
	int width = I->width;
	int i,j;
	uchar *pixel;
	CvMat* L=cvCreateMat(height,width,CV_32FC1);
	int up,left;
	int label=1;
	int count=0;
	map<int,int> eqvTable;	
	
	pixel=(uchar*)I->imageData;
	int lowest=-1;
	
	//-----------------LABEL THE PIXELS
	for(i=0;i<height;i++){
		for(j=0;j<width;j++){
			
			if(pixel[i*I->widthStep+j]==255){
				up=(i==0)?0:cvmGet(L,i-1,j);	//avoiding array out of bound indexing error
				left=(j==0)?0:cvmGet(L,i,j-1);
				//printf("(%d,%d) -> %d , %d\n",i,j,up,left);
				
				if(up==0 && left!=0){
					cvmSet(L,i,j,left);
				}
				if(up!=0 && left==0){
					cvmSet(L,i,j,up);
				}
				if(up!=0 && left!=0){
					cvmSet(L,i,j,up);
					if(up!=left){	
						eqvTable.insert(make_pair(left,up));
					}
				}
				else if(up==0 && left==0){
					label++;
					cvmSet(L,i,j,label);
				}
			
			}
		}
	}
	//-----------------
	
	//-----------------FIX THE TABLE
	int last;
	int prev_last,label_count=0;
	map<int,int> unique_labels;
	map<int,int>::iterator ul_iter;
	map<int,int>::iterator sec_iter;
	
	for( map<int,int>::iterator iter = eqvTable.begin(); iter != eqvTable.end(); ++iter ) {
		//printf("%d -> %d\n",(*iter).first,(*iter).second);
		last=(*iter).second;
		sec_iter=eqvTable.find((*iter).second);
		while(sec_iter!=eqvTable.end()){
			last=(*sec_iter).second;
			//printf("____ %d -> %d\n",(*sec_iter).first,(*sec_iter).second);
			sec_iter=eqvTable.find((*sec_iter).second);
		}
		
		if(prev_last!=last){
			prev_last=last;
			ul_iter=unique_labels.find(last);
			if(ul_iter==unique_labels.end()){
				label_count++;
				unique_labels.insert(make_pair(last,label_count));
			}
		}
		
		(*iter).second=last;
	}
	//-----------------
	
	//-----------------UPDATE THE LABEL MATRIX
	int temp;
	CvPoint pt;
	
	vector<CvPoint> blobs[label_count];
	for(i=0;i<L->height;i++){
		for(j=0;j<L->width;j++){
			temp=cvmGet(L,i,j);
			if(temp!=0){
				sec_iter=eqvTable.find(temp);
				if(sec_iter!=eqvTable.end()){
					//ul_iter=unique_labels.find((*sec_iter).second);					
					cvmSet(L,i,j,(*sec_iter).second);
				}
			}
		}
	}
	//----------------STORE info to "regions"
	for(i=0;i<height;i++){
		for(j=0;j<width;j++){
			temp=cvmGet(L,i,j);
			if(temp!=0){
				pt.x=i;
				pt.y=j;
				ul_iter=unique_labels.find(temp);	//[2,17,15] -map-> [1,2,3]
				temp=(*ul_iter).second;
				regions[temp-1].push_back(pt);		
			}
		}
	}
	return label_count;
}
