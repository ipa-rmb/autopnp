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
 * $Id: sendRequestFunction.c 7805 2014-03-13 09:54:35Z geisinger $
 */

/**
 * \file
 *         Source file for function sendRequest in component client.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "calculator/adv/client/include/sendRequestFunction.h"

#include "calculator/adv/client/include/sendRequestFunctionWrapper.h"
#include "calculator/adv/client/include/clientComponent.h"
#include "calculator/adv/client/include/clientComponentWrapper.h"
#include "calculator/adv/client/include/clientManifest.h"

#include "xme/core/logUtils.h"

// PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_C_INCLUDES) ENABLED START
#include "xme/hal/include/random.h"

#include <inttypes.h>
// PROTECTED REGION END

/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/

// PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_C_DEFINITIONS) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief  Variable holding the value of the required output port 'clientRequest'.
 *
 * \details If necessary initialize this in the init function.
 *          The value of this variable will be written to the port at the end of
 *          the step function.
 */
static calculator_topic_calculatorRequest_t
portClientRequestData;

// PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_C_VARIABLES) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

// PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_C_PROTOTYPES) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
calculator_adv_client_sendRequestFunction_init
(
    calculator_adv_client_clientComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_INITIALIZE_C) ENABLED START

    XME_UNUSED_PARAMETER(componentConfig);

    XME_LOG(XME_LOG_ALWAYS, "Client ready, going to send periodic requests to calculator server...\n");

    return XME_STATUS_SUCCESS;

    // PROTECTED REGION END
}

void
calculator_adv_client_sendRequestFunction_step
(
    calculator_adv_client_clientComponent_config_t* const componentConfig
)
{
    xme_status_t status[1];
    
    calculator_topic_calculatorRequest_t* portClientRequestDataPtr = &portClientRequestData;
    
    {
        // PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_STEP_C) ENABLED START
        char operation = '?';

        XME_UNUSED_PARAMETER(componentConfig);

        // Periodically send a request with operands from range -100..100
        portClientRequestData.operand1 = ((int32_t) xme_hal_random_randRange(0, 200)) - 100;
        portClientRequestData.operand2 = ((int32_t) xme_hal_random_randRange(0, 200)) - 100;
        portClientRequestData.operation = (uint8_t) xme_hal_random_randRange
        (
            (uint16_t) CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_ADD,
            (uint16_t) CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_DIVIDE
        );

        // Avoid division by zero
        if (0 == portClientRequestData.operand2 &&
            CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_DIVIDE == portClientRequestData.operation)
        {
            portClientRequestData.operand2 = 1;
        }

        switch (portClientRequestData.operation)
        {
        case CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_ADD:
            operation = '+';
            break;
        case CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_SUBTRACT:
            operation = '-';
            break;
        case CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_MULTIPLY:
            operation = '*';
            break;
        case CALCULATOR_TOPIC_CALCULATORREQUEST_OPERATION_DIVIDE:
            operation = '/';
            break;
        default:
            break;
        }

        XME_LOG
        (
            XME_LOG_ALWAYS,
            "sendRequestFunction: Sending request for %" PRIi32 " %c %" PRIi32 " = ?\n",
            portClientRequestData.operand1,
            operation,
            portClientRequestData.operand2
        );

        // PROTECTED REGION END
    }
    
    status[0] = calculator_adv_client_clientComponentWrapper_writePortClientRequest(portClientRequestDataPtr);
    
    {
        // PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_STEP_2_C) ENABLED START

        if (XME_STATUS_SUCCESS != status[0])
        {
            XME_LOG(XME_LOG_ERROR, "sendRequestFunction: Error sending request!\n");
        }

        // PROTECTED REGION END
    }
}

void
calculator_adv_client_sendRequestFunction_fini
(
    calculator_adv_client_clientComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_TERMINATE_C) ENABLED START
    
    // Nothing to do
    
    XME_UNUSED_PARAMETER(componentConfig);
    
    // PROTECTED REGION END
}

// PROTECTED REGION ID(CALCULATOR_ADV_CLIENT_SENDREQUESTFUNCTION_IMPLEMENTATION_C) ENABLED START

// PROTECTED REGION END
