
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

#ifndef __ITIA_HELIOS__CART_PATH_BASE__
#define __ITIA_HELIOS__CART_PATH_BASE__

# include <Eigen/Core>
# include <ros/console.h>
# include <itia_dynamics_core/itia_kin_solver_base.h>
# include <boost/function.hpp>
# include <itia_dynamics_core/cmotion_conversion.h>

namespace itia
{
namespace helios 
{

  class PathBase
  {

  public:
    enum MotionType {POLYNOMIAL, CPOLYNOMIAL};
    
    PathBase(const unsigned int& order,
             const unsigned int& dim,
             const itia::JMotion& init_jmotion, 
             boost::shared_ptr<itia::dynamics::KinSolverBase>& kin_solver)
    :
    m_kin_solver(kin_solver)
    {
      m_order=order;
      m_dim = dim;
      m_init_jmotion.resize(m_dim, m_order);
      m_init_jmotion.setZero();
      
      m_vector_length = 100;
      m_s.resize(m_vector_length);
      m_ps.resize(m_vector_length);
      
      m_length = 0;
      for (int idx = 0;idx<m_vector_length;idx++)
      {
        m_ps.at(idx) = (double)idx/(double)(m_vector_length-1);
        m_s.at(idx)  = (double)idx/(double)(m_vector_length-1);
      }
      if (m_dim != init_jmotion.rows())
      {
        ROS_ERROR("init_jmotion has wrong dimensions!!!");
        throw("init_jmotion has wrong dimensions!!!");
      }
      unsigned int tmp_size=init_jmotion.cols();
      if (tmp_size<order)
        m_init_jmotion.block(0, 0, m_dim, tmp_size) =init_jmotion;
      else if (tmp_size>order)
      {
        m_init_jmotion = init_jmotion.block(0, 0, m_dim, m_order);
        ROS_WARN_THROTTLE(1, "init_jmotion is bigger than order, skip undesired elements");
      }
      else
        m_init_jmotion =init_jmotion;
      
      m_final_jmotion = m_init_jmotion;
    };
    
    
    void setToolFrame(const std::string& tool_frame) {m_tool_frame = tool_frame;};
    void setBaseFrame(const std::string& base_frame) {m_base_frame = base_frame;};
    
    virtual itia::JMotion getJMotion( const double& s )= 0;
    virtual itia::CMotion getCMotion(const double& s, const std::string& tool_frame, const std::string& base_frame) = 0;
    virtual itia::CMotion getCMotion(const double& s) = 0;
    virtual double        getLength() {return m_length;};
    
    virtual bool          setFinalJMotion( const double& init_s,  const itia::JMotion& fin_jmotion, const bool& append = false, const MotionType& type = POLYNOMIAL)=0;
    virtual bool          setFinalCMotion( const double& init_s,  const itia::CMotion& fin_cmotion, const bool& append = false, const MotionType& type = POLYNOMIAL)=0;
    
    void getKinSolver(itia::dynamics::KinSolverBasePtr& ptr) {ptr = (m_kin_solver);};
    
  protected:
    unsigned int m_order;
    unsigned int m_dim;
    double m_length;
    itia::JMotion m_init_jmotion;
    itia::JMotion m_final_jmotion;
    MotionType m_type;
    itia::dynamics::KinSolverBasePtr m_kin_solver;
    std::string m_base_frame;
    std::string m_tool_frame;
    
    int m_vector_length;
    std::vector<double> m_s;                                // joint curvilinear abscissa
    std::vector<double> m_ps;                                // polynomial curvilinear abscissa
    
    
    virtual double computeJointLength()
    {
      double length = 1e-9;
      for (int idx = 0;idx<m_vector_length;idx++)
      {
        m_ps.at(idx) = (double)idx/(double)(m_vector_length-1);
        itia::JMotion jmotion = getJMotion(m_ps.at(idx));
        if (idx>0)
        {
          length += (jmotion.col(1)).norm() * (m_ps.at(idx)-m_ps.at(idx-1));
          m_s.at(idx) =length;
        }
      }
      
      return length;
    }
    
    virtual double convertJointLengthToPolyLength(const double& js)
    {
      double ps = 0;
      
      auto iter = std::lower_bound(m_s.begin(),m_s.end(), js);
      if (iter > m_s.begin())
        iter--;
      int idx = std::distance( m_s.begin(), iter );
      
      double ps_prev, ps_post, js_post, js_prev;
      if (idx == (m_vector_length-1))
      {
        ps_prev = m_ps.at(idx-1);
        ps_post = m_ps.at(idx);
        js_prev = m_s.at(idx-1);
        js_post = m_s.at(idx);
      }
      else
      {
        ps_prev = m_ps.at(idx);
        ps_post = m_ps.at(idx+1);
        js_prev = m_s.at(idx);
        js_post = m_s.at(idx+1);
      }
      
      if (std::abs(js_post-js_prev) <1e-9)
      {
        ps = ps_post;
      }
      else
      {
        ps = ps_prev + (ps_post-ps_prev)/(js_post-js_prev) *(js-js_prev);
      }
      return ps;
    }
    
    virtual double saturate(const double& s)
    {
      if (s<0)
      {
        ROS_WARN_THROTTLE(1, "curvilinear abscissa negative!! set equal to 0");
        return 0;
      }
      if (s>m_length)
      {
        ROS_WARN_THROTTLE(0.1, "curvilinear abscissa greater than path length!! set equal to %5.4f", m_length);
        return m_length;
      }
      return s;
    }
        
  };

  typedef boost::shared_ptr<itia::helios::PathBase> PathBasePtr;

}
}

#endif