// ControllerCtrl.cpp : Implementation of CTcControllerCtrl
#include "TcPch.h"
#pragma hdrstop

#include "ControllerW32.h"
#include "ControllerCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CControllerCtrl

CControllerCtrl::CControllerCtrl() 
	: ITcOCFCtrlImpl<CControllerCtrl, CControllerClassFactory>() 
{
}

CControllerCtrl::~CControllerCtrl()
{
}

