
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

#ifndef __ABSTRACT_POLYNOMIAL_MOTION_LAW_BASE__
#define __ABSTRACT_POLYNOMIAL_MOTION_LAW_BASE__

#include <Eigen/Core>
#include <ros/console.h>
# include <itia_helios/path_base.h>
namespace itia{
namespace helios{
  
  class MotionLawBase
  {
  protected:
    double m_duration;
    double m_start_time;
    double m_init_curv_absc;
    double m_fin_curv_absc;
    unsigned int m_order;
    bool m_planned;
    unsigned int m_curr_step;
    unsigned int m_nr_steps;
    double m_st;

    
    Eigen::VectorXd m_ub;
    Eigen::VectorXd m_lb;
    Eigen::MatrixXd m_A;
    Eigen::VectorXd m_b;
    Eigen::VectorXd m_state;
    PathBasePtr m_path;
    
    
    Eigen::VectorXd resizeToOrder(const Eigen::VectorXd& vector)
    {
      Eigen::VectorXd vector_out = Eigen::MatrixXd::Constant(m_order, 1, 0);
      if (vector.rows() < m_order)
        vector_out.block(0, 0, vector.rows(), 1) = vector;
      else
        vector_out = vector.block(0, 0, m_order, 1);
      return vector_out;
    }
  public:
    
    MotionLawBase(const unsigned int& order, 
                  const PathBasePtr& path, 
                  const Eigen::VectorXd& init_state, 
                  const double& init_curv_absc = 0,         
                  double start_time = 0
                  )
    {
      m_duration=0;
      m_order=order;
      m_A.resize(1,m_order);
      m_A.setZero();
      
      m_state=resizeToOrder(init_state);
      m_start_time = start_time;
      m_init_curv_absc = init_curv_absc;
      
      
      m_A.resize(order, order);
      m_A.setZero();
      m_b.resize(1);
      m_b.setZero();
      
      m_lb.resize(m_order);
      m_lb.setOnes();
      m_lb*=-1000;
      
      m_ub.resize(m_order);
      m_ub.setOnes();
      m_ub *= 1000;
      
      m_curr_step = 0;
      
    };
    
    
    
    virtual bool setFinalState(  const double& t, 
                            const double& fin_curv_absc, 
                            const double& duration) = 0;

    
    virtual Eigen::VectorXd getState( const unsigned int& step ) = 0;    
    virtual Eigen::VectorXd getState( const double& time ) = 0;    
    virtual unsigned int getCurrState() {return m_curr_step;};
    
    // compute next step
    virtual Eigen::VectorXd  update() = 0;
    
    // compute all steps (return the number of required steps)
    virtual unsigned int plan() = 0;
    
    
    virtual bool          getDuration(double* duration)
    {
      if (m_planned)
        *duration = m_duration;
      return m_planned;
    };
    
    virtual void            setLimit(const Eigen::MatrixXd& A,
                                     const Eigen::VectorXd& b,
                                     const Eigen::VectorXd& lb,
                                     const Eigen::VectorXd& ub)
    {
      m_A=A;
      m_b=b;
      m_lb=lb;
      m_ub=ub;      
    };
    
  };
  
  typedef boost::shared_ptr<itia::helios::MotionLawBase> MotionLawBasePtr;
}
}


#endif