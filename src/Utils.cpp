//
// Created by Wing Yee mak on 26/06/2018.
//

#include "Utils.h"

static int Utils::ConvertDToLane(double d){
    if(d < 0 || d> 12) {
        return -1;
    }
    if(d < 4) {
        return 0;
    } else if(d < 8) {
        return 1;
    } else {
        return 2;
    }
}
