/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id$
 */

/**
 * \file
 *         Defines function for creation of component type manifest
 *         for 'capabilitiesView'.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "autoPnP/adv/capabilitiesView/include/capabilitiesViewManifest.h"

#include "../include/capabilitiesViewComponentWrapper.h"
#include "autoPnP/adv/capabilitiesView/include/addComponentFunction.h"
#include "autoPnP/adv/capabilitiesView/include/addComponentFunctionWrapper.h"
#include "autoPnP/adv/capabilitiesView/include/capabilitiesViewComponent.h"
#include "autoPnP/adv/capabilitiesView/include/removeComponentFunction.h"
#include "autoPnP/adv/capabilitiesView/include/removeComponentFunctionWrapper.h"
#include "autoPnP/topic/autoPnPCommon.h"
#include "xme/core/log.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
autoPnP_adv_capabilitiesView_manifest_createComponentTypeManifest
(
    xme_core_componentManifest_t* componentManifest
)
{
    XME_CHECK
    (
        NULL != componentManifest,
        XME_STATUS_INVALID_PARAMETER
    );

    (void)xme_hal_mem_set(componentManifest, 0U, sizeof(xme_core_componentManifest_t));

    componentManifest->componentType = (xme_core_componentType_t)4097;
    componentManifest->componentWrapperInit = autoPnP_adv_capabilitiesView_capabilitiesViewComponentWrapper_init;
    componentManifest->componentWrapperReceivePort = autoPnP_adv_capabilitiesView_capabilitiesViewComponentWrapper_receivePort;
    componentManifest->componentWrapperFini = autoPnP_adv_capabilitiesView_capabilitiesViewComponentWrapper_fini;
    componentManifest->componentInit = (xme_core_componentManifest_componentInit_t)autoPnP_adv_capabilitiesView_capabilitiesViewComponent_init;
    (void)xme_hal_safeString_strncpy(componentManifest->name, "capabilitiesView", sizeof(componentManifest->name));
    componentManifest->configStructSize = sizeof(autoPnP_adv_capabilitiesView_capabilitiesViewComponent_config_t);

    {
        uint32_t functionArrayLength = sizeof(componentManifest->functionManifests) / sizeof(componentManifest->functionManifests[0]);

        // Function 'addComponent'
        {
            if (0U >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0U == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component type 'capabilitiesView' defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                       2,
                       functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
                
                functionManifest = &componentManifest->functionManifests[0];
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = (xme_hal_time_timeInterval_t)1ull;
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
                functionManifest->requiredPortIndicesLength = 1;
                functionManifest->requiredPortIndices[0] = (autoPnP_adv_capabilitiesView_capabilitiesViewComponentWrapper_internalPortId_t)AUTOPNP_ADV_CAPABILITIESVIEW_CAPABILITIESVIEWCOMPONENTWRAPPER_PORT_ADDCOMPONENT;
                functionManifest->optionalPortIndicesLength = 0;
                functionManifest->functionInit = (xme_core_exec_initCallback_t)autoPnP_adv_capabilitiesView_addComponentFunction_init;
                functionManifest->functionFini = (xme_core_exec_finiCallback_t)autoPnP_adv_capabilitiesView_addComponentFunction_fini;
                functionManifest->functionWrapperExecute = autoPnP_adv_capabilitiesView_addComponentFunctionWrapper_execute;
            }
        }
        // Function 'removeComponent'
        {
            if (1U >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (1U == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component type 'capabilitiesView' defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                       2,
                       functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
                
                functionManifest = &componentManifest->functionManifests[1];
                functionManifest->functionId = (xme_core_component_functionId_t)2;
                functionManifest->wcet = (xme_hal_time_timeInterval_t)1ull;
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
                functionManifest->requiredPortIndicesLength = 1;
                functionManifest->requiredPortIndices[0] = (autoPnP_adv_capabilitiesView_capabilitiesViewComponentWrapper_internalPortId_t)AUTOPNP_ADV_CAPABILITIESVIEW_CAPABILITIESVIEWCOMPONENTWRAPPER_PORT_REMOVECOMPONENT;
                functionManifest->optionalPortIndicesLength = 0;
                functionManifest->functionInit = (xme_core_exec_initCallback_t)autoPnP_adv_capabilitiesView_removeComponentFunction_init;
                functionManifest->functionFini = (xme_core_exec_finiCallback_t)autoPnP_adv_capabilitiesView_removeComponentFunction_fini;
                functionManifest->functionWrapperExecute = autoPnP_adv_capabilitiesView_removeComponentFunctionWrapper_execute;
            }
        }
    }

    {
        uint32_t portArrayLength;
        portArrayLength = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);

        // Subscription 'addComponent'
        {
            if (0 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component type 'capabilitiesView' defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        2,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[0];
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                portManifest->topic = AUTOPNP_TOPIC_ADD_COMPONENT; 
                portManifest->lowerConnectionBound = 0u;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                portManifest->queueSize = 10;
                portManifest->persistent = false;
        
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
            }
        }
        // Subscription 'removeComponent'
        {
            if (1 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (1 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component type 'capabilitiesView' defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        2,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[1];
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                portManifest->topic = AUTOPNP_TOPIC_REMOVE_COMPONENT; 
                portManifest->lowerConnectionBound = 0u;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                portManifest->queueSize = 10;
                portManifest->persistent = false;
        
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
            }
        }
    }
    return XME_STATUS_SUCCESS;
}
