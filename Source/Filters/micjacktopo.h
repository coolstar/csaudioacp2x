/*++

Copyright (c) Microsoft Corporation All Rights Reserved

Module Name:

    micjack.h

Abstract:

    Declaration of topology miniport for the mic (external: headphone).

--*/

#ifndef _CSAUDIOACP2X_MICJACKTOPO_H_
#define _CSAUDIOACP2X_MICJACKTOPO_H_

// Function declarations.
NTSTATUS
PropertyHandler_MicJackTopoFilter(_In_ PPCPROPERTY_REQUEST      PropertyRequest);

NTSTATUS PropertyHandler_MicJackTopology(_In_ PPCPROPERTY_REQUEST PropertyRequest);

#endif // _CSAUDIOACP2X_MICJACKTOPO_H_
