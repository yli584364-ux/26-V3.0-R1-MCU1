#ifndef _FLAGS_H_
#define _FLAGS_H_
#pragma once

#include "cmsis_os2.h"

extern osEventFlagsId_t flags_id;
void                    flags_create();

#endif // _FLAGS_H_