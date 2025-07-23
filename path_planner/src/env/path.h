/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Malte Hauff - initial API and implementation
 ********************************************************************************/

#pragma once
#include <boost/container/vector.hpp>

namespace adore
{
    struct Node_Lite
            {
                double x;
                double y;
                double psi;
                double s;   //used only for smoothing
            };
    typedef boost::container::vector<Node_Lite> TrajectoryVector;
}