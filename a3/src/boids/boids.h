#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum InitTypes {
     ZERO=0, RAMDOM=1
};

enum TimeIntegrationSchemes{
    EXPLICITEULER=0, SYMPLECTIC=1, EXPLICIT=2
};

enum MethodTypes {
        FREEFALL=0, COHESION=1, ALIGNMENT=2, SEPARATION=3, COLLISION_AVOIDANCE=4, LEADING=5,COLLABORATION_ADVERSARY=6
    };





template <class T, int dim>
class Boids
{
    
    typedef Matrix<T, Eigen::Dynamic, 1> VectorSig;
    typedef Matrix<T, dim, Eigen::Dynamic> VectorMlt;
    typedef Vector<T, dim> TV;
    typedef Matrix<T, dim, dim> TM;
    
    
private:
    VectorMlt positions;
    VectorMlt velocities;
    int n;
    bool update = false;
    float h, f_scaler, range;
    TV target_pos;
    bool obstacleOpen = false;

public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions(ZERO);
    }
    ~Boids() {}

    struct Obstacle
    {
        TV pos = TV(0.5f, 0.5f);
        float radius = 0.2f;
    } obstacle;

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions(InitTypes initVel)
    {
        positions = VectorMlt::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        
         switch (initVel)
        {
            case 0: velocities = VectorMlt::Zero(dim, n); break;
            case 1: velocities = VectorMlt::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); break;
            default : velocities = VectorMlt::Zero(dim, n); break;
        }
        
    }

    void updateBehavior(MethodTypes type, TimeIntegrationSchemes rule)
    {
        std::function<VectorMlt(VectorMlt)> method;
        if(!update)
            return;
        switch (type)
        {  
            case 0: method = std::bind(&Boids::freefall, this, std::placeholders::_1); break;
            case 1: method = std::bind(&Boids::cohesion, this, std::placeholders::_1);break;
            case 2: method = std::bind(&Boids::alignment, this, std::placeholders::_1); break;
            case 3: method = std::bind(&Boids::separation, this, std::placeholders::_1); break;
            case 4: method = std::bind(&Boids::collisionAvoidance, this, std::placeholders::_1); break;
            case 5: method = std::bind(&Boids::leading, this,  std::placeholders::_1); break;
            default: method = std::bind(&Boids::freefall, this, std::placeholders::_1); break;
        }

        switch (rule)
        {
            case 0: explicit_euler(method); break;
            case 1: symplectic_euler(method); break;
            case 2: explicit_euler(method); break;
            default: explicit_euler(method); break;
        }
        
        
    }

    void pause()
    {
        update = !update;
    }
    VectorMlt getPositions()
    {
        return positions;
    }

   void read_parameters(float h_read, float f_scaler_read, float range_read, Obstacle obstacle_read, bool drawCircles)
   {
       h = h_read;
       f_scaler = f_scaler_read;
       range = range_read;
       obstacle = obstacle_read;
       obstacleOpen = drawCircles;
       

   } 
   void getCursor(TV target_pos_read)
   {
       target_pos = target_pos_read;
   }


   VectorMlt freefall(VectorMlt positionsTemp)
   {
       TV g(0,9.8);
       VectorMlt f;
       f = g.replicate(1,n);
       return f;
   }

   VectorMlt cohesion(VectorMlt positionsTemp)
   {
       VectorMlt f =VectorMlt::Zero(dim, n);
       for (int i =0; i < positionsTemp.cols(); i++)
       {
           
           int neighborCount =0;
           VectorMlt neighbor = VectorMlt::Zero(dim, n);
           for (int j =0; j<positionsTemp.cols(); j++)
           {
               if((positionsTemp.col(i)-positionsTemp.col(j)).norm()<=range)
               {
                   neighborCount ++;
                   neighbor.col(j) = positionsTemp.col(j);
                   
               }
           }
           TV center = neighbor.rowwise().sum()/neighborCount;
           f.col(i) = (center - positionsTemp.col(i))*f_scaler - 0.2f * velocities.col(i);


       }
       return f;
   }


   VectorMlt separation(VectorMlt positionsTemp)
   {
       VectorMlt f = VectorMlt::Zero(dim, n);
       for(int i = 0; i < positionsTemp.cols(); i++)
       {
           for(int j =0; j<positionsTemp.cols(); j++)
           {
               if((positionsTemp.col(i)-positionsTemp.col(j)).norm() <= range)
               {
                   f_scaler = 1/pow((positionsTemp.col(i)-positionsTemp.col(j)+TV(1.0f,1.0f)).norm(),2);
                   f.col(i) = f.col(i) + (positionsTemp.col(i) - positionsTemp.col(j)).normalized() * f_scaler;
               }
           }
       }
       return f;
   }

   VectorMlt alignment(VectorMlt positionsTemp)
   {

       VectorMlt fVel =VectorMlt::Zero(dim, n);
       VectorMlt fPos = VectorMlt::Zero(dim, n);
       VectorMlt f = VectorMlt::Zero(dim,n);
       fPos = cohesion(positionsTemp);
       for( int i =0; i< positionsTemp.cols(); i++)
       {
           int neighborCount =0;
           VectorMlt neighbor = VectorMlt::Zero(dim,n);
           for(int j =0; j<positionsTemp.cols(); j++)
           {
               if((positionsTemp.col(i)-positionsTemp.col(j)).norm() <=range)
               {
                   neighborCount ++;
                   neighbor.col(j) = velocities.col(j);


               }

           }
           TV center = neighbor.rowwise().sum() / neighborCount;
           fVel.col(i) = f_scaler * (center - center.dot(velocities.col(i))* velocities.col(i).normalized());


       }

       return f = fPos + fVel;
       
   }



   void explicit_euler(std::function<VectorMlt(VectorMlt)> method)
   {
       VectorMlt positionsTemp = positions;
       positions = positions + h * velocities;
       VectorMlt f = method(positionsTemp);
       velocities =velocities+ h * f;
   }

   void symplectic_euler(std::function<VectorMlt(VectorMlt)> method)
   {
       positions =positions + h * velocities;
       VectorMlt positionsTemp = positions;
       VectorMlt f = method(positionsTemp);
       velocities = velocities + h *f;


   }
   void explicit_midpoint(std::function<VectorMlt(VectorMlt)> method)
   {
       VectorMlt positionsTemp = positions;
       VectorMlt positionsTempHalf = velocities * h/2 + positionsTemp;
       VectorMlt fTempHalf = method(positionsTemp);
       VectorMlt velocitiesTempHalf = h/2 * fTempHalf + velocities;
       positions = positions + velocitiesTempHalf * h;
       VectorMlt f = method(positionsTempHalf);
       velocities = velocities + f*h;


   }

   VectorMlt leading(VectorMlt positionsTemp)
   {
       VectorMlt f = VectorMlt::Zero(dim, n);
       VectorMlt fRep = VectorMlt::Zero(dim,n);
       VectorMlt fObs = VectorMlt::Zero(dim,n);
       f.col(0) = f_scaler * (target_pos - positionsTemp.col(0)) - 1.0f * velocities.col(0);
       for (int i =1; i < positionsTemp.cols(); i++)
       {
           if((positionsTemp.col(0) - positionsTemp.col(i)).norm()<=range)
           {
               f.col(i) += (positionsTemp.col(0) - positionsTemp.col(i)) + 2.0f * (velocities.col(0) - velocities.col(i));

           }
       }
       
       range = 0.05f;
       fRep = separation(positionsTemp);
       fRep.col(0) = TV(0.0f, 0.0f);
       if (obstacleOpen) fObs = collisionAvoidance(positionsTemp);
       return f= f + fRep + fObs;

       

       
   }

   VectorMlt collisionAvoidance(VectorMlt positionsTemp)
   {
       VectorMlt f = VectorMlt::Zero(dim, n);
       for(int i =0; i < positionsTemp.cols(); i++)
       {
           if((positionsTemp.col(i) - obstacle.pos).norm() <= obstacle.radius)
           {
               positions.col(i) = obstacle.pos +(positionsTemp.col(i) - obstacle.pos).normalized() * (obstacle.radius*1.01f);
               f_scaler = 1.0f / pow((positionsTemp.col(i) - obstacle.pos + TV(0.6f, 0.6f) ).norm(),2);
               f.col(i) = f_scaler * (positionsTemp.col(i) - obstacle.pos).normalized();

           }
       }
       return f;
   }

   


};
#endif
