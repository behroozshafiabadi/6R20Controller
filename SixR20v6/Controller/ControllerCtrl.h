///////////////////////////////////////////////////////////////////////////////
// ControllerCtrl.h

#ifndef __CONTROLLERCTRL_H__
#define __CONTROLLERCTRL_H__

#include <atlbase.h>
#include <atlcom.h>

#define CONTROLLERDRV_NAME "CONTROLLER"

#include "resource.h"       // main symbols
#include "ControllerW32.h"
#include "TcBase.h"
#include "ControllerClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CControllerCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CControllerCtrl, &CLSID_ControllerCtrl>
	, public IControllerCtrl
	, public ITcOCFCtrlImpl<CControllerCtrl, CControllerClassFactory>
{
public:
	CControllerCtrl();
	virtual ~CControllerCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_CONTROLLERCTRL)
DECLARE_NOT_AGGREGATABLE(CControllerCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CControllerCtrl)
	COM_INTERFACE_ENTRY(IControllerCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __CONTROLLERCTRL_H__
