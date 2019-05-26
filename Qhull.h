/****************************************************************************
**
** Copyright (c) 2008-2018 C.B. Barber. All rights reserved.
** $Id: //main/2015/qhull/src/libqhullcpp/Qhull.h#5 $$Change: 2549 $
** $DateTime: 2018/12/28 22:24:20 $$Author: bbarber $
**
****************************************************************************/

#ifndef QHULLCPP_H
#define QHULLCPP_H

#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullFacet.h"

namespace orgQhull {

/***
   Compile qhullcpp and libqhull with the same compiler.  setjmp() and longjmp() must be the same.

   #define QHULL_NO_STL
      Do not supply conversions to STL
      Coordinates.h requires <vector>.  It could be rewritten for another vector class such as QList
   #define QHULL_USES_QT
      Supply conversions to QT
      qhulltest requires QT.  It is defined in RoadTest.h

  #define QHULL_ASSERT
      Defined by QhullError.h
      It invokes assert()
*/

#//!\name Used here
    class QhullFacetList;
    class QhullPoints;
    class QhullQh;
    class RboxPoints;

#//!\name Defined here
    class Qhull;

//! Interface to Qhull from C++
class Qhull {

private:
#//!\name Members and friends
    QhullQh *           qh_qh;          //! qhT for this instance
    Coordinates         origin_point;   //! origin for qh_qh->hull_dim.  Set by runQhull()
    bool                run_called;     //! True at start of runQhull.  Errors if call again.
    Coordinates         feasible_point;  //! feasible point for half-space intersection (alternative to qh.feasible_string for qh.feasible_point)

public:
#//!\name Constructors
                        Qhull();      //!< call runQhull() next
                        Qhull(const char *inputComment2, int pointDimension, int pointCount, const realT *pointCoordinates, const char *qhullCommand2);
                        ~Qhull() throw();
private:                //! Disable copy constructor and assignment.  Qhull owns QhullQh.
                        Qhull(const Qhull &);
    Qhull &             operator=(const Qhull &);

private:
    void                allocateQhullQh();

public:

#//!\name GetSet
    void                checkIfQhullInitialized();
    bool                initialized() const { return (qh_qh->hull_dim>0); }


#//!\name Methods
    double              area();
    void                runQhull(const RboxPoints &rboxPoints, const char *qhullCommand2);
    void                runQhull(const char *inputComment2, int pointDimension, int pointCount, const realT *pointCoordinates, const char *qhullCommand2);
    double              volume();

#//!\name Helpers
private:
    void                initializeFeasiblePoint(int hulldim);
};//Qhull

}//namespace orgQhull

#endif // QHULLCPP_H
