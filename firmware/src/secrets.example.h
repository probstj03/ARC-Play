#pragma once
// Copy to secrets.h (not versioned) and fill in your networks.
// Used only on first boot; afterwards the list lives in NVS and is managed
// from ARC-Server ("Bekannte Netzwerke") or via the serial command "wifi".
#include "arc_net.h"
static const arc::net::DefaultNetwork kDefaultWifi[] = {
    {"MeinWLAN", "passwort", 10},
};
static const char* const kArcNetPassword = "arc-net-2026";
