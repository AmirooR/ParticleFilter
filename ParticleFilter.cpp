  // File:          ParticleFilter.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <time.h>

//#include "Object.h"

using namespace std;
/*
#include <../c/webots/differential_wheels.h>
#include <../c/webots/distance_sensor.h>
#include <../c/webots/led.h>
#include <../c/webots/robot.h>
*/
#include <webots/DifferentialWheels.hpp>


// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define log(...) cerr<<__VA_ARGS__
#define MAX(x,y) (x>y?x:y)
#define MIN(x,y) (x<y?x:y)

#define LSPD 40.0//low speed
#define NSPD 100.0//normal speed
#define HSPD 200.0//high speed

#define WDIST .02
#define FDIST .02

#define WAIT 30
#define MISSWALL 200
#define TRAPED 100
#define RESET 1000
#define TIME_STEP 128//64


#define WHEEL_RADIUS 0.0205
#define RADIUS 0.035
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)

#define RNG_RESOLUTION 1000

#define PI 3.1415926535897932
#define PI2	6.2831853071795864
#define SQRT2PI 2.50662827
#define MAPPING_SIZE 300

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
//

static const char *distNames[8] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};
static const char *leds_names[10] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7",
  "led8", "led9"
};

int num_particle;

class State
{
public:
	double x;
	double y;
	double theta;

	State(double x1=0,double y1=0,double theta1=0):x(x1),y(y1),theta(theta1)
	{
	}
/*
	State(const State& state):x(state.x),y(state.y),theta(state.theta)
	{
	}
*/
	void update (double dx,double dy,double dtheta) 
	{
		x += dx;
		y += dy;
		theta += dtheta;
		
		if (theta > PI)
		{
			theta -= PI2;
		} 
		else if (theta <= -PI)
		{
			theta += PI2;
		} 
	}

	friend ostream& operator<<(ostream& out,State s)
	{
		out<<"X: "<<s.x<<" Y: "<<s.y<<" theta: "<<s.theta*180/PI<<endl;
		return out;
	}
	State operator+(State op)
	{
		return State(x + op.x, y + op.y, theta + op.theta);
	}

	State operator*(double op)
	{
		return State(x*op, y*op, theta*op);
	}
};

class Particle:public State
{
public:
	Particle(double x,double y,double theta,double nw = 1.0/num_particle):State(x,y,theta)
	{
		w = nw;
	}

	Particle(State st,double nw = 1.0/num_particle):State(st.x,st.y,st.theta)
	{
		w = nw;
	}
	friend ostream& operator<<(ostream& out,Particle p)
	{
		out<<"X: "<<p.x<<" Y: "<<p.y<<" theta: "<<p.theta*180/PI<< " Weight: "<<p.w<<endl;
		return out;
	}

	double w;
};

class Map
{
private:
	int resolution, xMax, yMax;
	char** map;
	double** dist;

	void calcDist()
	{
		double mind;
		int r = (int)(.071*resolution)+1;
//		FILE *distfile = fopen("distfile.txt","w");
		
		for (int y = 0; y<yMax; y++)
		{
			for (int x = 0; x<xMax; x++)
			{
				if (map[y][x] == '1')
				{
					dist[y][x] = 0;
				}
				else
				{
					mind = .08;
					for (int i = MAX(y-r,0); i < MIN(y+r,yMax); i++)
					{
						for (int j = MAX(x-r,0); j < MIN(x+r,xMax); j++)
						{
							if (map[i][j] == '1')
							{
								mind = MIN(mind,pow(pow((y-i), 2) + pow((x-j), 2), .5) / resolution);
							}
						}
					}
					dist[y][x] = MIN(mind,.075);
				}
//				fprintf(distfile, "%.4f\t", dist[y][x]);
			}
//			fprintf(distfile, "\n");
		}
//		fclose(distfile);
	}

public:
	Map(const char* filename)
	{
		FILE* file;
		char temp[10];
		if ((file = fopen(filename, "r")) == NULL)
		{
			cerr<<"Cannot open map file..."<<endl;
		}
		fscanf(file, "%d %d %d", &resolution, &yMax, &xMax);
		fgets(temp, 10, file);
			
		dist = new double*[yMax];
		map = new char*[yMax];
		for(int y=yMax-1; y>=0; y--)
		{
			dist[y] = new double[xMax+2];
			map[y] = new char[xMax+2];
			fgets(map[y], xMax+2, file);
//			cout<<map[y];
		}
		fclose(file);
		calcDist();
	}
	
	~Map()
	{
		for(int y=0; y<yMax; y++)
		{
			delete map[y];
			delete dist[y];
		}
		delete map;
		delete dist;
	}
	
	double getXMax()
	{
		return (double)xMax / (double)resolution;
	}
	
	double getYMax()
	{
		return (double)yMax / (double)resolution;
	}
	
	double getMinDistance(double xx,double yy)
	{
		int x = (int)(xx*resolution+.5);
		int y = (int)(yy*resolution+.5);
		if (x < xMax && x >=0 && y < yMax && y >= 0)
		{
			/*if(dist[y][x] < 0)
			{
				cout<<"\n*\n*\nError dist: "<<dist[y][x]<<endl;
			}*/
			return MAX(0,dist[y][x]);
		}
		return 10.0;
	}

	bool isOccupied(State state)
	{
		int x = (int)(state.x*resolution+.5);
		int y = (int)(state.y*resolution+.5);
		
		if (x < xMax && x >=0 && y < yMax && y >= 0)
		{
			return (dist[y][x] < .035);
		}
//			cout <<x<<"\t"<<y<<endl;
		return true;
	}
};

class ParticleFilter : public DifferentialWheels 
{

	// You may need to define your own functions or variables, like
	LED *led[10];
	DistanceSensor* ds[8];

	State approx;
	FILE* movie;
	Map* map;
	vector<Particle>* particles;
	vector<State> sensorPoses; 
	double a1,a2,a3,a4;
	double zmax,zhit,znowall,zrandom,sigmaHit;
	double max_x,max_y;
	double a_fast,a_slow,w_slow,w_fast;

	int **hit;
	int **totalMap;
	int **pathMap;
	int global;

	public:

	// ParticleFilter constructor
	ParticleFilter(): DifferentialWheels() {

		// You should insert a getDevice-like function in order to get the
		// instance of a device of the robot. Something like:
		map = new Map("map.map");

		hit = new int*[MAPPING_SIZE];
		totalMap = new int*[MAPPING_SIZE];
		pathMap = new int*[MAPPING_SIZE];
		for(int ii=0;ii<MAPPING_SIZE;ii++)
		{
			hit[ii] = new int[MAPPING_SIZE];
			totalMap[ii] = new int[MAPPING_SIZE];
			pathMap[ii] = new int[MAPPING_SIZE];
			for(int jj=0;jj<MAPPING_SIZE;jj++)
			{
				hit[ii][jj] = 0;
				totalMap[ii][jj] = -1;
				pathMap[ii][jj] = 0;
			}
		}
		init();

		/*      
			State pX(2.0,3.0,PI);
			State pU(2.04,2.8,PI-0.15);
			State nU(2.1,2.81,PI-0.2);
			*/
		for( int i=0;i<10;i++)
		{
			led[i] = getLED(leds_names[i]);
		}

		for( int i=0;i<8;i++)
		{
			ds[i] = getDistanceSensor(distNames[i]);
			ds[i]->enable(TIME_STEP);
		}
		enableEncoders(TIME_STEP);
		setEncoders(0.0,0.0);


	}

	// ParticleFilter destructor
	virtual ~ParticleFilter() {

		// Enter here exit cleanup code

	}

	void init()
	{
		srand(time(0));
		a1 = 1.0;
		a3 = 0.05;//0.05;
		a2 = 0.05; 
		a4 = 0.003;
		num_particle = 5000;//1000;
		particles = new vector<Particle>();
		a_slow = 0.01;
		a_fast = 0.9;
		w_slow = 0.0;
		w_fast = 0.0;
		zmax = 0.055;
		zhit = 1.0;
		znowall = 0.5;
		zrandom = 0.0125;
		sigmaHit = 0.015;

		max_x = map->getXMax();
		max_y = map->getYMax();

		State sensorPose;

		/*sensorPose.x     =  0.033436777;
		  sensorPose.y     = -0.010343207;
		  sensorPose.theta = -0.29999999996640769;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     =  0.022530689693073834;
		  sensorPose.y     = -0.026783726812271973;
		  sensorPose.theta = -0.87142857131499862;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     =  0;
		  sensorPose.y     = -0.035;
		  sensorPose.theta = -1.5707963267948966;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     = -0.029706926397973173;
		  sensorPose.y     = -0.018506715645554311;
		  sensorPose.theta = -2.5844497965195483;
		  sensorPoses.push_back(sensorPose);


		  sensorPose.x     = -0.029706926397973173;
		  sensorPose.y     = 0.018506715645554311;
		  sensorPose.theta = 2.5844497965195483;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     = 0;
		  sensorPose.y     = 0.035;
		  sensorPose.theta = 1.5707963267948966;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     = 0.022530689693073834;
		  sensorPose.y     = 0.026783726812271973;
		  sensorPose.theta = 0.87142857131499862;
		  sensorPoses.push_back(sensorPose);

		  sensorPose.x     = 0.033436777;
		  sensorPose.y     = 0.010343207;
		  sensorPose.theta = 0.29999999996640769;
		  sensorPoses.push_back(sensorPose);
		  */

		sensorPose.y     = 0.033436777;
		sensorPose.x     = 0.010343207;
		sensorPose.theta = PI/2.0 - 0.29999999996640769;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = 0.022530689693073834;
		sensorPose.x     = 0.026783726812271973;
		sensorPose.theta = PI/2.0-0.87142857131499862;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = 0;
		sensorPose.x     = 0.035;
		sensorPose.theta = 0.0;//-1.5707963267948966;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = -0.029706926397973173;
		sensorPose.x     = 0.018506715645554311;
		sensorPose.theta = PI/2.0-2.5844497965195483;
		sensorPoses.push_back(sensorPose);


		sensorPose.y     = -0.029706926397973173;
		sensorPose.x     = -0.018506715645554311;
		sensorPose.theta = -1.5*PI+2.5844497965195483;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = 0;
		sensorPose.x     = -0.035;
		sensorPose.theta = PI;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = 0.022530689693073834;
		sensorPose.x     = -0.026783726812271973;
		sensorPose.theta = PI/2.0+0.87142857131499862;
		sensorPoses.push_back(sensorPose);

		sensorPose.y     = 0.033436777;
		sensorPose.x     = -0.010343207;
		sensorPose.theta = PI/2.0+0.29999999996640769;
		sensorPoses.push_back(sensorPose);


		for(int i=0;i<num_particle;i++)
		{
			particles->push_back(sample_particle_map());
		}

	}

	void mapping(vector<double> z,State state)
	{
		state.x = state.x-0.7;
		state.y = state.y-0.7;
		static int num_called = 0;
		int x_index = (int)(0.5+(MAPPING_SIZE/2) + state.x*100.0);
		int y_index = (int)(0.5+(MAPPING_SIZE/2) + state.y*100.0);
		if(totalMap[y_index][x_index] == -1)
		{
			totalMap[y_index][x_index] = 0;
		}
		totalMap[y_index][x_index]++;
		pathMap[y_index][x_index]++;
		//-------
		//if(totalMap[y_index][x_index] %2)
		//{
		//hit[y_index][x_index]++;
		//}
		//--------
		for(int i=0;i<8;i++)
		{
			for(double k = -0.02; k<= z[i]; k+= 0.01)
			{

				double xz = state.x + (k+RADIUS)*cos(state.theta+sensorPoses[i].theta - PI/2.0);
				double yz = state.y + (k+RADIUS)*sin(state.theta+sensorPoses[i].theta - PI/2.0);
				y_index = (int)(0.5 + (MAPPING_SIZE/2) + yz*100.0);
				x_index = (int)(0.5 + (MAPPING_SIZE/2) + xz*100.0);
				totalMap[y_index][x_index]++;
				if(z[i] < zmax && (z[i] - k) < 0.01)
				{
					hit[y_index][x_index]++;
				}
			}


		}

		if(num_called % 20 == 19)
		{
			FILE* fmap_total = fopen("MAPTotal.txt","w");
			FILE* fmap_hit = fopen("MAPHit.txt","w");
			FILE* fmap_path = fopen("MAPPath.txt","w");
			for(int i=0;i<MAPPING_SIZE;i++)
			{
				for(int j=0;j<MAPPING_SIZE;j++)
				{
					fprintf(fmap_total,"%d ",totalMap[i][j]);
					fprintf(fmap_hit,"%d ",hit[i][j]);
					fprintf(fmap_path,"%d ",pathMap[i][j]);
				}
				fprintf(fmap_total,"\n");
				fprintf(fmap_hit,"\n");
				fprintf(fmap_path,"\n");
			}
			fclose(fmap_hit);
			fclose(fmap_total);
			fclose(fmap_path);

			cerr<<"Map Written -------------------"<<endl;
		}
		num_called++;
	}

	Particle sample_particle_map()
	{
		//	cout<<"\t\tin sample_particle_map function "<<max_x<<"\t"<<max_y<<endl;
		while(true)
		{
			double rx = rand01() * max_x;
			double ry = rand01() * max_y;
			double rtheta = (rand01()-.5) * PI2;//rand01() * PI;//
			//double rtheta = approx.theta+(rand01()-.5);//((rand()%16)-7)*PI2/16;
		
			Particle p(rx,ry,rtheta, 1.0/num_particle);

			if(!map->isOccupied(p))
			{
				return p;
			}

		}

	}

	inline double rand01()
	{
		return ((double)(rand()%RNG_RESOLUTION))/((double)RNG_RESOLUTION+1);
	}

	Particle getRandomParticle(double sum_weigths,vector<Particle>* barParticles) //TODO:input barParticle?
	{
		//cerr<<"getRandomParticle"<<endl;
		double sum = 0.0;
		double my_prob = rand01()*sum_weigths;
		int index = rand()%num_particle;
		//cerr<<"my_prob"<<my_prob<<endl;
		for(int i=index; i != index-1; i = (i+1)%num_particle)
		{

			Particle par = (*barParticles)[i];
			sum += par.w;
			//cerr<<"Iteration "<<i<<" sum "<<sum<<endl;
			if(my_prob < sum)
			{
				Particle t = par;
				t.update(sample(0.01),sample(0.01),sample(0.05));
				if(!map->isOccupied(t))
					return t;
				return par;
			}
		}
		cerr<<"Last Particle\t--------------"<<endl;
		return (*particles)[index];
	}

	inline double gaussian(double mean,double sigma,double x)
	{
		return exp(-0.5*(x-mean)*(x-mean)/(sigma*sigma));//(sigma*SQRT2PI);
	}

	double liklihoodFiled(vector<double> z,State state)
	{
		double q = 1.0;
		if (map->isOccupied(state))
			return 0.0;
		for(int i=0;i<8;i++)
		{

			double xz = state.x + (RADIUS + z[i])*cos(state.theta + sensorPoses[i].theta - PI/2.0);
			double yz = state.y + (RADIUS + z[i])*sin(state.theta + sensorPoses[i].theta - PI/2.0);
			double dist = map->getMinDistance(xz,yz);

			if(z[i] < zmax)
			{
				q *= (zhit*gaussian(0,sigmaHit,dist) + zrandom/zmax);
			}
			/*else if(dist < zmax)
			   q *= znowall;*/
      /*
			if(dist - z[i] > 0.04)//((z[i] < zmax) && (dist > zmax))
			  q *= .1;
			else if(z[i] - dist > 0.04)//((z[i] > zmax) && (dist < zmax))
			  q *= .2;		
			else 
			  q *= .8;
*/
		}
		return q;
	}

	inline double sample(double a)
	{
		double sum = 0.0;
		for(int i=0;i<12;i++)
			sum += rand01()*2.0*a - a;
		return sum/2.0;
	}

	State sampleMotionOdometryMap(State& prevU,State& thisU,State& prevX)
	//State sampleMotionOdometryMap(double dtran, double drot, State& prevX)
	{
		double dxU = thisU.x - prevU.x;
		double dyU = thisU.y - prevU.y;
		double dRot1 = atan2(dyU, dxU) - prevU.theta;
		double dtrans = sqrt(dxU*dxU + dyU * dyU);
		double dRot2 = thisU.theta - prevU.theta -  dRot1;

		//State thisState(0,0,0);	
		//for(int iii =1; iii<20; iii++)//do
		{
			double tRot1 = dRot1 - sample(a1*fabs(dRot1) + a2*dtrans);
			double tTrans = dtrans - sample(a3*dtrans + a4*(fabs(dRot1) + fabs(dRot2)));
			double tRot2 = dRot2 - sample(a1*fabs(dRot2) + a2*dtrans);



			State thisState = prevX;
			thisState.update(tTrans*cos(prevX.theta + tRot1),
					tTrans*sin(prevX.theta + tRot1),
					tRot1 + tRot2);
			/*(prevX.x + tTrans*cos(prevX.theta+tRot1),
			  prevX.y + tTrans*sin(prevX.theta + tRot1),
			  prevX.theta + tRot1 + tRot2);*/
			/*      thisState.x = prevX.x + tTrans*cos(prevX.theta+tRot1);
				thisState.y = prevX.y + tTrans*sin(prevX.theta + tRot1);
				thisState.theta = prevX.theta + tRot1 + tRot2;
				*/      
			/*double tRot = drot + sample(a1*fabs(drot) + a2*dtran);
			  double tTrans = dtran + sample(a3*dtran + a4*fabs(drot));
			  thisState = prevX;
			  thisState.update(tTrans*cos(prevX.theta+tRot), tTrans*sin(prevX.theta + tRot), tRot);
			  */
			if (!(map->isOccupied(thisState)))
				return thisState;
		}//while(map->isOccupied(thisState));
		//cout<<"Error.............................\t"<<global<<endl;
		global++;
		return sample_particle_map();//thisState;//sample_particle_map();
	}

	void augmented_MCL(State prevU,State nU,vector<double> z)
	//void augmented_MCL(double dtran, double drot,vector<double> z)
	{
		static int num_movie = 1;
		if(num_movie %1000 == 1)
		{
			if(num_movie > 1)
			{
				fclose(movie);
				cout<<" .......................... End File .......................... "<<endl;
			}
			char name[20];
			sprintf(name,"saved/movie_%d.dat",num_movie/1000);
			movie = fopen(name,"w");
		}

		//    cout<<"\t\tBegin MCL";
		int m;
		double w_avg = 0.0;
		State mean;
		vector<Particle>* thisParticles = new vector<Particle>();
		vector<Particle>* barParticles = new vector<Particle>();
		global = 0;
		for(m=0;m<num_particle;m++)
		{
			//	  cout<<"Before Odometry "<<m<<endl;
			State nX = sampleMotionOdometryMap(prevU,nU,(*particles)[m]);
			//State nX = sampleMotionOdometryMap(dtran, drot,(*particles)[m]);
			double wm = liklihoodFiled(z,nX);
			//cerr<<"wm "<<wm<<endl;
			Particle bP(nX,wm);
			barParticles->push_back(bP);
			//	  if(num_movie % 2 == 1)
			if(m % 5 == 0)
				fprintf(movie,"%.4f %.4f %.4f %.4f ",bP.x,bP.y,bP.theta,bP.w);
			w_avg = w_avg + wm/num_particle;
			//	  cout<<"After Odometry "<<m<<endl;
		}
		/*w_slow = w_slow + a_slow*(w_avg - w_slow);
		  w_fast = w_fast + a_fast*(w_avg - w_fast);*/
		//   int sum_many = 0;
		for(m=0;m<num_particle;m++)
		{
			/*	  int many = (int)(.5+((*barParticles)[m].w / w_avg));

				  for (int jjj = 0; jjj < many; jjj++)
				  {
				  sum_many++;
				  if(sum_many >= num_particle)
				  break;
				  thisParticles->push_back((*barParticles)[m]);		
				  }
				  if(sum_many >= num_particle)
				  break;
				  */		
			mean = mean + (*barParticles)[m];
			//	  cout<<"Before resample "<<m<<endl;
			//double my_prob = rand01();
			//double val = 1.0 - w_fast/w_slow;
			if(  rand01() <0.01)
			{
				Particle p = sample_particle_map();
				thisParticles->push_back(p);
			}
			else
			{
				Particle par = getRandomParticle(w_avg*num_particle,barParticles);
				thisParticles->push_back(par);
			}


			//	  cout<<"after resample "<<m<<endl;
		}
		cerr<<num_movie<<":\tParticle\t\t"<<mean*(1.0/num_particle)<<endl;

		delete particles;
		delete barParticles;
		particles = thisParticles;
		fprintf(movie,"%.4f %.4f %.4f\n",approx.x,approx.y,approx.theta);
		num_movie++;
		//	cout<<"\tEnd of MCL"<<endl;
	}


	vector<double> readDistSensors() const
	{  
		double reading;
		vector<double> answer;
		for(unsigned sensor = 0;  sensor < 8; ++sensor)
		{
			reading = ds[sensor]->getValue();

			if(reading > 825)
			{
				answer.push_back(-6.1312e-006 * reading + 0.0201);
			}
			// A reading between 403 and 941, represents a range between 2cm and 3cm
			// roughly
			else if(reading > 140)
			{
				answer.push_back(-2.6277e-005 * reading + 0.0367);
			}
			// A reading between 0 and 403, represents a range larger than 3cm
			else
			{
				answer.push_back(-3e-04 * reading + 0.075);
			}
			//      cerr << "zer from sensor #" << sensor << "=" << answer[sensor] << endl;

		}

		return answer;
	}

	// User defined function for initializing and running
	// the ParticleFilter class

	void run(){
		vector<double> distances;
		bool leftWall = false;
		bool rightWall = false;

		char status[20] = {0};
		State pstate,prev(1.33,0.07,0.0), state(1.33,0.07,0.0);
		double pl, pr, l = 0.0, r = 0.0, dl, dr, da;
		//    double x=0, y=0, theta=0;

		int free, seeWall, loopCounter = 0;
		int amin = 0;
    int wait = WAIT;
    
		// Main n		cout<<"Run Controller..."<<endl;
		do {
			//		State pstate(state);
			//blink_leds();
			// Read the sensors:
			// Enter here functions to read sensor data, like:
			//  double val = distanceSensor->getValue();

			// Process sensor data here

			// Enter here functions to send actuator commands, like:
			if(amin == 0)
			{
				led[5]->set(1);
				amin = 1;
			}
			else
			{
				led[5]->set(0);
				amin = 0;
			}

			pl = l;
			pr = r;
			l = getLeftEncoder();
			r = getRightEncoder();
			dl = (l-pl) / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
			dr = (r-pr) / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
			da = (dr - dl) *.9 / AXLE_LENGTH; // delta orientation in radian

			//	  x += sin(theta)*(dr + dl)/2;
			//      y += cos(theta)*(dr + dl)/2;
			//      theta += da;

			pstate = state;
			state.update(cos(pstate.theta)*(dr + dl)/2, sin(pstate.theta)*(dr + dl)/2, da);
			//state = sampleMotionOdometryMap((dr + dl)/2, da,state);

			approx = state;
			distances = readDistSensors();

			if(loopCounter % 10 == 0)
			{

				augmented_MCL(prev,state,distances);
				//double move = sqrt((state.x-prev.x)*(state.x-prev.x) + (state.y-prev.y)*(state.y-prev.y));
				//augmented_MCL(move, state.theta - prev.theta , distances);//augmented_MCL((dr + dl)/2, da, distances);
				prev = state;
			}
			mapping(distances,state);
			//augmented_MCL(pstate, state, distances);

			//  ************************************************************    


			if (distances[2] < FDIST)
			{
				//cout<<"\t\tright wall"<<endl;
				rightWall = true;
				seeWall = loopCounter;
			}
			else if(leftWall == true)
			{
				rightWall = false;
			}

			if (distances[5] < FDIST)
			{
				//cout<<"\t\tleft wall"<<endl;
				leftWall = true;
				seeWall = loopCounter;
			} 
			else if(rightWall == true)
			{
				leftWall = false;
			}

			if (loopCounter - seeWall > MISSWALL)
			{
				cout<<"miss wall"<<endl;
				rightWall = false;
				leftWall = false;
			}

			if (!(distances[0] < WDIST && distances[0] - distances[7] < 1 && distances[0] - distances[7] > -1) &&
					!(distances[1] < WDIST && distances[1] - distances[6] < 1 && distances[1] - distances[6] > -1))
			{
				free = loopCounter;
			}

			if (loopCounter - free > TRAPED)
			{
				//        rightWall = false;
				//        leftWall = false;
				cout<<"traped"<<endl;
				loopCounter = RESET-WAIT;
			}

			if (RESET-loopCounter <= wait)
			{
				sprintf(status,"reseting...");
        if(rightWall == true)
          setSpeed(-NSPD, NSPD);
        else
          setSpeed(NSPD, -NSPD);

			}
			else if (distances[0] < WDIST && !leftWall)// && distances[0] < distances[7])
			{
				sprintf(status,"fast left");
				setSpeed(-NSPD, NSPD);
			} 
			else if (distances[7] < WDIST && !rightWall)
			{
				sprintf(status,"fast right");
				setSpeed(NSPD, -NSPD);
			} 
			else if (distances[1] < WDIST && !leftWall)// && distances[1] < distances[6])
			{
				sprintf(status,"slow left");
				setSpeed(-LSPD, NSPD);
			} 
			else if (distances[6] < WDIST && !rightWall)
			{
				sprintf(status,"slow right");
				setSpeed(NSPD, -LSPD);
			} 
			else if (rightWall && distances[2] > FDIST)
			{
				sprintf(status,"follow right");
				setSpeed(NSPD, LSPD);
			} 
			else if (leftWall && distances[5] > FDIST)
			{
				sprintf(status,"follow left");
				setSpeed(LSPD, NSPD);
			} 
			else
			{
				if(rightWall)
					sprintf(status,"right wall");
				else if(leftWall)
					sprintf(status,"left wall");
				else
					sprintf(status,"forward");

				setSpeed(HSPD, HSPD);
			}

			/*  for(int i=0;i<8;i++)
			    {
			    cout<<distances[i]<<"\t";
			    }
			    cout<<endl;
			    */
			//delay(1);
			if (loopCounter == RESET)
			{
			        wait = WAIT/2 + (rand()%(WAIT));
				loopCounter = 0;
				rightWall = false;
				leftWall = false;
			}
			else
			{
				loopCounter++;
			}
			if(loopCounter % 10 == 0)
			{
				cout<<status<<"\t\t"<<loopCounter<<"\t"<<state<<endl;
			}
			//        cout<<"\t\t"<<loopCounter<<"\t"<<dl<<"\t"<<dr<<"\t"<<da<<endl;
			// Perform a simulation step of 64 milliseconds
			// and leave the loop when the simulation is over
		} while (step(TIME_STEP) != -1);
	}
};

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  cout<<"controller of the e-puck robot by Morad & Amir..."<<endl;
  ParticleFilter* controller = new ParticleFilter();
  controller->run();
  delete controller;
  return 0;
}
