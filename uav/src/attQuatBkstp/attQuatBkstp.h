// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Bkstp.h
 * \brief Class defining a Bkstp
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef ATTQUATBKSTP_H
#define ATTQUATBKSTP_H

#include <ControlLaw.h>

namespace flair {
namespace gui {
class LayoutPosition;
}
}

class attQuatBkstp_impl;

namespace flair {
namespace filter {
/*! \class Bkstp
*
* \brief Class defining a Bkstp
*/
class attQuatBkstp : public ControlLaw {
  friend class ::attQuatBkstp_impl;

public:
  /*!
  * \brief Constructor
  *
  * Construct a Bkstp at given position. \n
  * The Bkstp will automatically be child of position->getLayout() Layout. After
  *calling this function,
  * position will be deleted as it is no longer usefull. \n
  *
  * \param position position to display settings
  * \param name name
  */
  attQuatBkstp(const gui::LayoutPosition *position, std::string name);

  /*!
  * \brief Destructor
  *
  */
  ~attQuatBkstp();

  /*!
  * \brief Reset integral
  *
  */
  void Reset(void);
  
   /*!
  * \brief Get intergral part
  *
  * \return current integral part
  */
  float GetIntegral(void) const;


  /*!
  * \brief Set input values
  *
  * \param p proportional value
  * \param d derivative value
  */
void SetValues(float q0,        float q1,        float q2,       float q3,
               float q0_d,      float q1_d,      float q2_d,     float q3_d,
               float omega_x,   float omega_y,   float omega_z);

  /*!
  * \brief Use default plot
  *
  * Plot the output values at position. \n
  * Plot consists of 4 curves: proportional part,
  * derivative part, integral part and
  * the sum of the three. \n
  * After calling this function, position will be deleted as it is no longer
  *usefull. \n
  *
  * \param position position to display plot
  */
  void UseDefaultPlot(const gui::LayoutPosition *position);

private:
  /*!
  * \brief Update using provided datas
  *
  * Reimplemented from IODevice.
  *
  * \param data data from the parent to process
  */
  void UpdateFrom(const core::io_data *data);

  attQuatBkstp_impl *pimpl_;
};
} // end namespace filter
} // end namespace flair
#endif // Bkstp_H
