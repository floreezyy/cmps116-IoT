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
/// @file    GUIPropertySchemeStorage.h
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Laura Bieker
/// @date    Mon, 20.07.2009
/// @version $Id$
///
//
/****************************************************************************/
#ifndef GUIPropertySchemeStorage_h
#define GUIPropertySchemeStorage_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <fx.h>
#include "GUIPropertyScheme.h"


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIPropertySchemeStorage
 * @brief Base class for coloring. Allows changing the used colors and sets
 *  the used color in dependence to a value
 */
template<class T>
class GUIPropertySchemeStorage {
public:
    /// @brief Constructor
    GUIPropertySchemeStorage() : myActiveScheme(0) { }

    /// @brief Destructor
    virtual ~GUIPropertySchemeStorage() { }

    /// @brief Fills the given combobox with the names of available colorings
    void fill(FXComboBox& cb) {
        for (typename std::vector<T>::iterator i = mySchemes.begin(); i != mySchemes.end(); ++i) {
            cb.appendItem((*i).getName().c_str());
        }
        cb.setCurrentItem((FXint)myActiveScheme);
    }

    void setActive(int scheme) {
        if (scheme < (int)mySchemes.size()) {
            myActiveScheme = scheme;
        }
    }

    int getActive() const {
        return myActiveScheme;
    }

    T& getScheme() {
        return mySchemes[myActiveScheme];
    }

    const T& getScheme() const {
        return mySchemes[myActiveScheme];
    }

    const std::vector<T>& getSchemes() const {
        return mySchemes;
    }

    T* getSchemeByName(std::string name) {
        for (typename std::vector<T>::iterator i = mySchemes.begin(); i != mySchemes.end(); ++i) {
            if ((*i).getName() == name) {
                return &(*i);
            }
        }
        return 0;
    }

    void save(OutputDevice& dev) const {
        for (typename std::vector<T>::const_iterator i = mySchemes.begin(); i != mySchemes.end(); ++i) {
            i->save(dev);
        }
    }

    bool operator==(const GUIPropertySchemeStorage& c) const {
        return myActiveScheme == c.myActiveScheme && mySchemes == c.mySchemes;
    }


    void addScheme(T scheme) {
        mySchemes.push_back(scheme);
    }

    int size() const {
        return (int)mySchemes.size();
    }


protected:
    int myActiveScheme;
    std::vector<T> mySchemes;

};

typedef GUIPropertySchemeStorage<GUIColorScheme> GUIColorer;
typedef GUIPropertySchemeStorage<GUIScaleScheme> GUIScaler;

#endif

/****************************************************************************/
