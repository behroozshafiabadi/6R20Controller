﻿///////////////////////////////////////////////////////////////////////////////
// Generator.cpp
#include "TcPch.h"
#pragma hdrstop

#include "Generator.h"
#include "Traj7Seg.h"
#include "GlobalData.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
DEFINE_THIS_FILE()


extern GlobalClass global;
///////////////////////////////////////////////////////////////////////////////
// CGenerator
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Collection of interfaces implemented by module CGenerator
BEGIN_INTERFACE_MAP(CGenerator)
	INTERFACE_ENTRY_ITCOMOBJECT()
	INTERFACE_ENTRY(IID_ITcADI, ITcADI)
	INTERFACE_ENTRY(IID_ITcWatchSource, ITcWatchSource)
///<AutoGeneratedContent id="InterfaceMap">
	INTERFACE_ENTRY(IID_ITcCyclic, ITcCyclic)
///</AutoGeneratedContent>
END_INTERFACE_MAP()

IMPLEMENT_ITCOMOBJECT(CGenerator)
IMPLEMENT_ITCOMOBJECT_SETSTATE_LOCKOP2(CGenerator)
IMPLEMENT_ITCADI(CGenerator)
IMPLEMENT_ITCWATCHSOURCE(CGenerator)


///////////////////////////////////////////////////////////////////////////////
// Set parameters of CGenerator 
BEGIN_SETOBJPARA_MAP(CGenerator)
	SETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="SetObjectParameterMap">
	SETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	SETOBJPARA_VALUE(PID_GeneratorParameter, m_Parameter)
	SETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CGenerator 
BEGIN_GETOBJPARA_MAP(CGenerator)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	GETOBJPARA_VALUE(PID_GeneratorParameter, m_Parameter)
	GETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
END_GETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get watch entries of CGenerator
BEGIN_OBJPARAWATCH_MAP(CGenerator)
	OBJPARAWATCH_DATAAREA_MAP()
///<AutoGeneratedContent id="ObjectParameterWatchMap">
///</AutoGeneratedContent>
END_OBJPARAWATCH_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get data area members of CGenerator
BEGIN_OBJDATAAREA_MAP(CGenerator)
///<AutoGeneratedContent id="ObjectDataAreaMap">
	OBJDATAAREA_VALUE(ADI_GeneratorInputs, m_Inputs)
	OBJDATAAREA_VALUE(ADI_GeneratorOutputs, m_Outputs)
///</AutoGeneratedContent>
END_OBJDATAAREA_MAP()

char GUI_GetNextCMD1;
Traj7Seg trajectory;
TrajectoryPointList<double> targetPoints[6];
int index_point = 0;
double actualPos[6];
double targetPos[8];
//double toolParam[8] = { 1,0,0,0,0,0,0,0 };
double t1, t2, t3;
int i = 0;
///////////////////////////////////////////////////////////////////////////////
CGenerator::CGenerator()
	: m_Trace(m_TraceLevelMax, m_spSrv)
	, m_counter(0)
{
///<AutoGeneratedContent id="MemberInitialization">
	m_TraceLevelMax = tlAlways;
	memset(&m_Parameter, 0, sizeof(m_Parameter));
	memset(&m_Inputs, 0, sizeof(m_Inputs));
	memset(&m_Outputs, 0, sizeof(m_Outputs));
///</AutoGeneratedContent>
}

///////////////////////////////////////////////////////////////////////////////
CGenerator::~CGenerator()
{
}


///////////////////////////////////////////////////////////////////////////////
// State Transitions 
///////////////////////////////////////////////////////////////////////////////
IMPLEMENT_ITCOMOBJECT_SETOBJSTATE_IP_PI(CGenerator)

///////////////////////////////////////////////////////////////////////////////
// State transition from PREOP to SAFEOP
//
// Initialize input parameters 
// Allocate memory
HRESULT CGenerator::SetObjStatePS(PTComInitDataHdr pInitData)
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;
	IMPLEMENT_ITCOMOBJECT_EVALUATE_INITDATA(pInitData);

	// TODO: Add initialization code

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to OP
//
// Register with other TwinCAT objects
HRESULT CGenerator::SetObjStateSO()
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;

	// TODO: Add any additional initialization


	// If following call is successful the CycleUpdate method will be called, 
	// possibly even before method has been left.
	hr = FAILED(hr) ? hr : AddModuleToCaller();

	// Cleanup if transition failed at some stage
	if (FAILED(hr))
	{
		RemoveModuleFromCaller();
	}

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from OP to SAFEOP
HRESULT CGenerator::SetObjStateOS()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;

	RemoveModuleFromCaller();

	// TODO: Add any additional deinitialization

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to PREOP
HRESULT CGenerator::SetObjStateSP()
{
	HRESULT hr = S_OK;
	m_Trace.Log(tlVerbose, FENTERA);

	// TODO: Add deinitialization code

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///<AutoGeneratedContent id="ImplementationOf_ITcCyclic">
HRESULT CGenerator::CycleUpdate(ITcTask* ipTask, ITcUnknown* ipCaller, ULONG_PTR context)
{
	HRESULT hr = S_OK;
	if (global.GUI_GetNextCMD == 1) {
		global.GUI_GetNextCMD = 0;
		index_point = 0;  // شاخص صف داده تولید شده است
		for (i = 0; i < 6; i++) {
			// PulsToDegFactor1 در مورد 
			// انکدر باید باید در یک عدد ثابت ضرب شود تا تبدیل به درجه شود. این عدد ثابت با توجه به موتور و گیربکس متغییر است
			// در مورد انکدر : یکی دو سیم متصل موتور است که شمارنده ای است که با هر دور چرخش موتور 524000 عدد را می شمارد که به نوعی نشان از دقت موتور است
			actualPos[i] = (static_cast<double>(global.ActualPosition[i])) * trajectory.PulsToDegFactor1[i]; // به ازای هر موتور یک عدد است trajectory.PulsToDegFactor1 
			targetPos[i] = /*actualPos[i] +*/ (global.GUI_TargetPosition[i]);
			targetPoints[i].clearAll();
			/*
           مشخص میشود xyz rpy یک آرایه هشت تایی است که به صورت  targetPos در تمام حالت ها 
			*/
			//targetPos[i] = (double)(global.GUI_TargetPosition[i]);
		}
		targetPos[6] = global.GUI_TargetPosition[6];
		targetPos[7] = global.GUI_TargetPosition[7];
		switch (global.GUI_Manager)
		{
		case 8: //PTP
			trajectory.PTPList(actualPos, targetPos, targetPoints);
			break;
		case 10: //PTP Position XYZ ABC
			//for (i = 0; i < 6; i++) {
			//	actualPos[i] = (actualPos[i] * M_PI) / 180.0; //to radian
			//}
			trajectory.PTPCartesian(actualPos, targetPos , targetPoints);
			break;
		case 12: //CIRC
			for (i = 0; i < 6; i++) {
				actualPos[i] = (actualPos[i] * M_PI) / 180.0; //to radian
			}
			//trajectory.GetCartPos(actualPos, toolParam, getCartPosOut);	
			trajectory.CIRC(actualPos, trajectory.point2Circ, trajectory.point3Circ, targetPoints);
			break;
		case 16: //LIN
			for (i = 0; i < 6; i++) {
				actualPos[i] = (actualPos[i] * M_PI) / 180.0; //to radian
			}
			//trajectory.GetCartPos(actualPos, toolParam, getCartPosOut);	
			trajectory.LIN(actualPos, targetPos, trajectory.toolParamGlobal, targetPoints);
			break;
		case 98: // Home
			targetPos[0] = 0;
			targetPos[1] = 0;
			targetPos[2] = 0;
			targetPos[3] = 0;
			targetPos[4] = 0;
			targetPos[5] = 0;
			targetPos[6] = 50;
			targetPos[7] = 0;
			trajectory.PTPList(actualPos, targetPos, targetPoints);
				break;
		case 100:
			//nothing
			break;
		}
		while (targetPoints[0].TrajLength > index_point) {
			for (i = 0; i < 6; i++) {
				global.set_dataPoint(i, (long)(targetPoints[i].q[index_point] * (1.0 / trajectory.PulsToDegFactor1[i])));
			}
			/*t1 = targetPoints[0].q[0] *(1.0 / trajectory.PulsToDegFactor1[0]);
			t2 = targetPoints[1].q[0] *(1.0 / trajectory.PulsToDegFactor1[1]);
			t3 = targetPoints[2].q[0] *(1.0 / trajectory.PulsToDegFactor1[2]);*/
			index_point++;
		}
		global.GUI_GetNextCMD = 1;
	}
		
	return hr;
}
///</AutoGeneratedContent>

///////////////////////////////////////////////////////////////////////////////
HRESULT CGenerator::AddModuleToCaller()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;
	if (m_spCyclicCaller.HasOID())
	{
		if (SUCCEEDED_DBG(hr = m_spSrv->TcQuerySmartObjectInterface(m_spCyclicCaller)))
		{
			if (FAILED(hr = m_spCyclicCaller->AddModule(m_spCyclicCaller, THIS_CAST(ITcCyclic))))
			{
				m_spCyclicCaller = NULL;
			}
		}
	}
	else
	{
		hr = ADS_E_INVALIDOBJID;
		SUCCEEDED_DBGT(hr, "Invalid OID specified for caller task");
	}

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
VOID CGenerator::RemoveModuleFromCaller()
{
	m_Trace.Log(tlVerbose, FENTERA);

	if (m_spCyclicCaller)
	{
		m_spCyclicCaller->RemoveModule(m_spCyclicCaller);
	}
	m_spCyclicCaller = NULL;

	m_Trace.Log(tlVerbose, FLEAVEA);
}

