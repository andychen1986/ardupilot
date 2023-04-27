#include "mode.h"
#include "Rover.h"

bool ModeDigging::_enter()
{
    _init = 0;
    return true;
}

void ModeDigging::_exit()
{

}

void ModeDigging::update()
{
    if(!_init){
        gcs().send_text(MAV_SEVERITY_NOTICE, "Enter Mode Digging!");
        _init = 1;
    }
}

