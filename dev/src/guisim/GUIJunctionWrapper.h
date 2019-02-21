/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    GUIJunctionWrapper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 1 Jul 2003
/// @version $Id$
///
// Holds geometrical values for a junction
/****************************************************************************/
#ifndef GUIJunctionWrapper_h
#define GUIJunctionWrapper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utility>
#include <utils/geom/PositionVector.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/globjects/GUIGlObject.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSNet;
class MSJunction;
#ifdef HAVE_OSG
namespace osg {
class Geometry;
}
#endif


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIJunctionWrapper
 *
 * As MSJunctions do not have a graphical representation but a complex
 *  inheritance tree, this class is used to encapsulate the geometry of an
 *  abstract junction and to be used as a gl-object.
 *
 * In the case the represented junction's shape is empty, the boundary
 *  is computed using the junction's position to which an offset of 1m to each
 *  side is added.
 */
class GUIJunctionWrapper : public GUIGlObject {
public:
    /** @brief Constructor
     * @param[in] junction The represented junction
     */
    GUIJunctionWrapper(MSJunction& junction);


    /// @brief Destructor
    virtual ~GUIJunctionWrapper();



    /// @name inherited from GUIGlObject
    //@{

    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app,
                                       GUISUMOAbstractView& parent);


    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    GUIParameterTableWindow* getParameterWindow(
        GUIMainWindow& app, GUISUMOAbstractView& parent);


    /** @brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const;


    /** @brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    //@}

    /** @brief Returns the boundary of the junction
     * @return This junction's boundary
     */
    Boundary getBoundary() const {
        return myBoundary;
    }

    /// @brief whether this is an inner junction (a waiting spot for crossing a "real" junction)
    bool isInternal() const {
        return myIsInternal;
    }

    /** @brief Returns the represented junction
     * @return The junction itself
     */
    const MSJunction& getJunction() const {
        return myJunction;
    }


#ifdef HAVE_OSG
    void setGeometry(osg::Geometry* geom) {
        myGeom = geom;
    }

    void updateColor(const GUIVisualizationSettings& s);
#endif

private:
    double getColorValue(const GUIVisualizationSettings& s) const;

private:
    /// @brief A reference to the represented junction
    MSJunction& myJunction;

    /// @brief The maximum size (in either x-, or y-dimension) for determining whether to draw or not
    double myMaxSize;

    /// @brief The represented junction's boundary
    Boundary myBoundary;

    /// @brief whether this wraps an instance of MSInternalJunction
    bool myIsInternal;

    /// @brief whether this junction has only waterways as incoming and outgoing edges
    bool myAmWaterway;

#ifdef HAVE_OSG
    osg::Geometry* myGeom;
#endif


private:
    /// @brief Invalidated copy constructor.
    GUIJunctionWrapper(const GUIJunctionWrapper&);

    /// @brief Invalidated assignment operator.
    GUIJunctionWrapper& operator=(const GUIJunctionWrapper&);

};


#endif

/****************************************************************************/

