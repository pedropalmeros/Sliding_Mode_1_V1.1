// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file zBackstepping.h
 * \brief Class defining a Pid for Thrust
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2014/11/07
 * \version 4.0
 */

#ifndef ZBACKSTEPPING_H
#define ZBACKSTEPPING_H

#include <ControlLaw.h>

namespace flair {
namespace gui {
class LayoutPosition;
}
}

class zBackstepping_impl;

namespace flair {
namespace filter {
/*! \class zBackstepping
*
* \brief Class defining a Pid for Thrust.\n
* This Pid as an extra offset for compensating gravity.
*/
class zBackstepping : public ControlLaw {
  friend class ::zBackstepping_impl;

public:
  /*!
  * \brief Constructor
  *
  * Construct a zBackstepping at given position
  * The zBackstepping will automatically be child of position->getLayout() Layout.
  *After calling this function,
  * position will be deleted as it is no longer usefull. \n
  *
  * \param position position to display settings
  * \param name name
  */
  zBackstepping(const gui::LayoutPosition *position, std::string name);

  /*!
  * \brief Destructor
  *
  */
  ~zBackstepping();

  /*!
  * \brief Reset the control law
  * equivalent to:
  * ResetI();
  * SetDefaultOffset();
  * SetValues(0,0);
  * 
  */
  void Reset(void);
  
  /*!
  * \brief Reset integral to 0
  *
  */
  void ResetI(void);

  /*!
  * \brief Set offset to a specified value
  *
  * \param value desired value
  */
  void SetOffset(float value);
  
  /*!
  * \brief Set offset to specified value in ground station
  *
  */
  void SetDefaultOffset(void);

  /*!
  * \brief Get offset
  *
  * \return current offset
  */
  float GetOffset(void) const;
  
  /*!
  * \brief Get intergral part
  *
  * \return current integral part
  */
  float GetIntegral(void) const;

  /*!
  * \brief Step up the offset according to specified value in ground station
  *
  * \return false if offset is at its maximum (1) value, true otherwise
  */
  bool OffsetStepUp(void);

  /*!
  * \brief Step down the offset according to specified value in ground station
  *
  * \return false if offset is at its minimum (specified in ground station)
  *value, true otherwise
  */
  bool OffsetStepDown(void);

  /*!
  * \brief Set input values
  *
  * \param p proportional value
  * \param d derivative value
  */
  void SetValues(float pos, float ref_pos, float vel, float ref_vel);

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

  zBackstepping_impl *pimpl_;
};
} // end namespace filter
} // end namespace flair
#endif // zBackstepping_H
