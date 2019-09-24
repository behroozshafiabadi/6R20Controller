///////////////////////////////////////////////////////////////////////////////
// Main.cpp
#include "TcPch.h"
#pragma hdrstop

#include "Main.h"
#include "MainAds.h"
#include "Traj7Seg.h"
#include "GlobalData.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
DEFINE_THIS_FILE()

///////////////////////////////////////////////////////////////////////////////
// CMain
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Collection of interfaces implemented by module CMain
BEGIN_INTERFACE_MAP(CMain)
	INTERFACE_ENTRY_ITCOMOBJECT()
	INTERFACE_ENTRY(IID_ITcADI, ITcADI)
	INTERFACE_ENTRY(IID_ITcWatchSource, ITcWatchSource)
///<AutoGeneratedContent id="InterfaceMap">
	INTERFACE_ENTRY(IID_ITcCyclic, ITcCyclic)
///</AutoGeneratedContent>
END_INTERFACE_MAP()

IMPLEMENT_ITCOMOBJECT(CMain)
IMPLEMENT_ITCOMOBJECT_SETSTATE_LOCKOP2(CMain)
IMPLEMENT_ITCADI(CMain)
IMPLEMENT_ITCWATCHSOURCE(CMain)

///////////////////////////////////////////////////////////////////////////////
// Set parameters of CMain 
BEGIN_SETOBJPARA_MAP(CMain)
	SETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="SetObjectParameterMap">
	SETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	SETOBJPARA_VALUE(PID_MainDefaultAdsPort, m_DefaultAdsPort)
	SETOBJPARA_VALUE(PID_Ctx_AdsPort, m_ContextAdsPort)
	SETOBJPARA_VALUE(PID_MainCounter, m_Counter)
	SETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CMain 
BEGIN_GETOBJPARA_MAP(CMain)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	GETOBJPARA_VALUE(PID_MainDefaultAdsPort, m_DefaultAdsPort)
	GETOBJPARA_VALUE(PID_Ctx_AdsPort, m_ContextAdsPort)
	GETOBJPARA_VALUE(PID_MainCounter, m_Counter)
	GETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
	GETOBJPARA_TYPE_CODE(PID_MainAdsPort, WORD, *p = AmsGetPort())
END_GETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get watch entries of CMain
BEGIN_OBJPARAWATCH_MAP(CMain)
	OBJPARAWATCH_DATAAREA_MAP()
///<AutoGeneratedContent id="ObjectParameterWatchMap">
	OBJPARAWATCH_VALUE(PID_MainCounter, m_Counter)
///</AutoGeneratedContent>
END_OBJPARAWATCH_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get data area members of CMain
BEGIN_OBJDATAAREA_MAP(CMain)
///<AutoGeneratedContent id="ObjectDataAreaMap">
	OBJDATAAREA_VALUE(ADI_MainInputs, m_Inputs)
	OBJDATAAREA_VALUE(ADI_MainOutputs, m_Outputs)
///</AutoGeneratedContent>
END_OBJDATAAREA_MAP()



///////////////////////////////////////////////////////////////////////////////
CMain::CMain()
	: m_Trace(m_TraceLevelMax, m_spSrv)
{
///<AutoGeneratedContent id="MemberInitialization">
	m_TraceLevelMax = tlAlways;
	m_DefaultAdsPort = 0;
	m_ContextAdsPort = 0;
	m_Counter = 0;
	memset(&m_Inputs, 0, sizeof(m_Inputs));
	memset(&m_Outputs, 0, sizeof(m_Outputs));
///</AutoGeneratedContent>
}

///////////////////////////////////////////////////////////////////////////////
CMain::~CMain()
{
}

///////////////////////////////////////////////////////////////////////////////
// State Transitions 
///////////////////////////////////////////////////////////////////////////////
IMPLEMENT_ITCOMOBJECT_SETOBJSTATE_IP_PI(CMain)

///////////////////////////////////////////////////////////////////////////////
// State transition from PREOP to SAFEOP
//
// Initialize input parameters 
// Allocate memory
HRESULT CMain::SetObjStatePS(PTComInitDataHdr pInitData)
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;
	IMPLEMENT_ITCOMOBJECT_EVALUATE_INITDATA(pInitData);

	hr = SUCCEEDED(hr) ? InitAmsPort(m_spSrv, m_DefaultAdsPort) : hr;

	// cleanup on failure
	if (FAILED(hr))
	{
		ShutdownAmsPort();
	}
	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to OP
//
// Register with other TwinCAT objects
HRESULT CMain::SetObjStateSO()
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;

	// TODO: Add any additional initialization


	// If following call is successful the CycleUpdate method will be called, 
	// possibly even before this method has been left.
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
HRESULT CMain::SetObjStateOS()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;

	RemoveModuleFromCaller();

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to PREOP
HRESULT CMain::SetObjStateSP()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;
	ShutdownAmsPort();

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}
GlobalClass global;
Traj7Seg traj7Seg;
double JogCurrentSpeed[6] = { 0, 0 , 0, 0, 0, 0 };
double JogMaxSpeed = 100000000;
double JogAcceleration = 10000000;
double JogMaxSpeedCart = 1000;// 100000000;
double JogAccelerationCart = 100;// 10000000;
double tmpMovement[6] = { 0, 0 , 0, 0, 0, 0 };
double actualPos1[6];
double currentPos[8], jogNext[8];
double rpy[3];
double quat[4];
double Q[6];
double Traj7Seg::QbaseGlobal[8] = { 1,0,0,0,0,0,0,0 };
double Traj7Seg::toolParamGlobal[8] = { 1,0,0,0,0,0,0,0 };
double Traj7Seg::point2Circ[8] = { 0,0,0,0,0,0,0,0 };
double Traj7Seg::point3Circ[4] = { 0,0,0,-1 };
///<AutoGeneratedContent id="ImplementationOf_ITcCyclic">
HRESULT CMain::CycleUpdate(ITcTask* ipTask, ITcUnknown* ipCaller, ULONG_PTR context)
{
	HRESULT hr = S_OK;

	// handle pending ADS indications and confirmations
	CheckOrders();


	// TODO: Additional evaluation of input from ADS indication or confirmations

	// TODO: Submit your ADS requests 
	ULONGLONG cnt = 0;
	if (SUCCEEDED(ipTask->GetCycleCounter(&cnt)))
	{
		if (cnt % 100 == 0)
		{
			SubmitAdsReadReq();
		}
	}
	global.GUI_Manager = m_Inputs.GUI_Manager;

	for (int i = 0; i < 6; i++) {
		global.ActualPosition[i] = m_Inputs.ActualPosition[i];
	}
	
	for (int i = 0; i < 8; i++) {
		global.GUI_TargetPosition[i] = static_cast<double>(m_Inputs.GUI_TargetPosition[i]);// because of hanking
	}
	//}
	switch (m_Inputs.GUI_Manager)
	{
	case 2: // run motors
		for (int i = 0; i < 6; i++) {
			m_Outputs.TargetPosition[i] = m_Inputs.ActualPosition[i];
			m_Outputs.ControlWord[i] = 15;
		}
		m_Outputs.ModOfOperation = 8;
		m_Inputs.GUI_Manager = 100;
		break;
	case 4: // enables motors
		for (int i = 0; i < 6; i++) {
			m_Outputs.ControlWord[i] = 7;
		}
		m_Inputs.GUI_Manager = 100;
		break;
	case 8: //PTP
		if (global.GUI_GetNextCMD == 0)
			m_Inputs.GUI_Manager = 100;
		break;
	case 10: //PTP Position XYZ ABC
		if (global.GUI_GetNextCMD == 0)
			m_Inputs.GUI_Manager = 100;
		break;
	//case 12: //CIRC
	//	if (global.GUI_GetNextCMD == 0)
	//		m_Inputs.GUI_Manager = 100;
	//	break;
	case 16: // CIRC OR LIN
		if (global.GUI_GetNextCMD == 0)
			m_Inputs.GUI_Manager = 100;
		break;
	case 21: //get P2 for circ 
		for (int i = 0; i < 8; i++) {
			traj7Seg.point2Circ[i] = static_cast<double>(m_Inputs.GUI_TargetPosition[i]);
		}
			m_Inputs.GUI_Manager = 23;
		break;
	case 22: //get P3 for circ 
		for (int i = 0; i < 4; i++) {
			traj7Seg.point3Circ[i] = static_cast<double>(m_Inputs.GUI_TargetPosition[i]);
		}
		m_Inputs.GUI_Manager = 23;
		break;
	case 64: // Jog Joint
		for (int i = 0; i < 6; i++) {
			if (!m_Inputs.GUI_StopingJog && m_Inputs.GUI_MSelect[i])
			{
				JogCurrentSpeed[i] = JogCurrentSpeed[i] + 0.001 * JogAcceleration;
			}
			else {
				JogCurrentSpeed[i] = JogCurrentSpeed[i] - 0.01 * JogAcceleration;
			}
			if (JogCurrentSpeed[i] > JogMaxSpeed)
			{
				JogCurrentSpeed[i] = JogMaxSpeed;
			}
			else if (JogCurrentSpeed[i] <= 0)
			{
				JogCurrentSpeed[i] = 0;
			}

		}
		for (int i = 0; i < 6; i++) {
			tmpMovement[i] = 0.001 * JogCurrentSpeed[i];
			m_Outputs.TargetPosition[i] = m_Inputs.ActualPosition[i] + tmpMovement[i] * m_Inputs.GUI_JogDirection;
		}
		break;
	case 65: // Jog Cartesian		
		for (int i = 0; i < 6; i++) {
			if (!m_Inputs.GUI_StopingJog && m_Inputs.GUI_MSelect[i])
			{
				JogCurrentSpeed[i] = JogCurrentSpeed[i] + 0.001 * JogAccelerationCart;
			}
			else {
				JogCurrentSpeed[i] = JogCurrentSpeed[i] - 0.01 * JogAccelerationCart;
			}
			if (JogCurrentSpeed[i] > JogMaxSpeedCart)
			{
				JogCurrentSpeed[i] = JogMaxSpeedCart;
			}
			else if (JogCurrentSpeed[i] <= 0)
			{
				JogCurrentSpeed[i] = 0;
			}
		}
		for (int i = 0; i < 6; i++) {
			actualPos1[i] = ((double)(m_Inputs.ActualPosition[i] * traj7Seg.PulsToDegFactor1[i])* M_PI) / 180.0;
		}
		// BEGIN TEMP
		/*actualPos1[2] = 1;
		actualPos1[3] = 1;
		actualPos1[4] = 1;
		actualPos1[5] = 1;*/
		// END TEMP
		traj7Seg.GetCartPos(actualPos1, traj7Seg.toolParamGlobal, currentPos);		
		traj7Seg.toEulerianAngle(currentPos, rpy);
		 currentPos[5] = currentPos[5] + 0.001 * JogCurrentSpeed[0] * m_Inputs.GUI_JogDirection;
		 currentPos[6] = currentPos[6] + 0.001 * JogCurrentSpeed[1] * m_Inputs.GUI_JogDirection;
		 currentPos[7] = currentPos[7] + 0.001 * JogCurrentSpeed[2] * m_Inputs.GUI_JogDirection;
		 rpy[0] = rpy[0] + 0.001 * JogCurrentSpeed[3] * m_Inputs.GUI_JogDirection;
		 rpy[1] = rpy[1] + 0.001 * JogCurrentSpeed[4] * m_Inputs.GUI_JogDirection;
		 rpy[2] = rpy[2] + 0.001 * JogCurrentSpeed[5] * m_Inputs.GUI_JogDirection;		 
		 traj7Seg.toQuaternion(rpy[0] * M_PI / 180.0, rpy[1] * M_PI / 180.0, rpy[2] * M_PI / 180.0, quat);
		 for (int i = 0; i < 4; i++)
			 currentPos[i] = quat[i];
		 traj7Seg.Inversekinematic(currentPos, traj7Seg.QbaseGlobal, traj7Seg.toolParamGlobal, actualPos1, Q);
		 for (int i= 0;i < 6; i++)
		 {
			 m_Outputs.TargetPosition[i] =Q[i] * (180.0 / M_PI)* (1.0 / traj7Seg.PulsToDegFactor1[i]);
		 }
		break;
	case 96: // Set Tool Frame
		for (int i = 0; i < 8; i++) {
			traj7Seg.toolParamGlobal[i] = static_cast<double>(m_Inputs.GUI_TargetPosition[i]);
		}
		m_Inputs.GUI_Manager = 100;
		break;
	case 97: // Set Base Frame
		for (int i = 0; i < 8; i++) {
			traj7Seg.QbaseGlobal[i] = static_cast<double>(m_Inputs.GUI_TargetPosition[i]);
		}
		m_Inputs.GUI_Manager = 100;
		break;
	case 98: // Home
		if (global.GUI_GetNextCMD == 0)
			m_Inputs.GUI_Manager = 100;
		break;
	case 99: // clear alarms
		for (int i = 0; i < 6; i++) {
			m_Outputs.ControlWord[i] = 128;
		}
		m_Inputs.GUI_Manager = 100;
		break;
	case 100:
		//nothing
		break;
	default:
		break;
	}
	if (global.GUI_GetNextCMD == 1 && !global.m0.empty())
		m_Outputs.TargetPosition[0] = global.get_dataPoint(0);
	if (global.GUI_GetNextCMD == 1 && !global.m1.empty())
		m_Outputs.TargetPosition[1] = global.get_dataPoint(1);
	if (global.GUI_GetNextCMD == 1 && !global.m2.empty())
		m_Outputs.TargetPosition[2] = global.get_dataPoint(2);
	if (global.GUI_GetNextCMD == 1 && !global.m3.empty())
		m_Outputs.TargetPosition[3] = global.get_dataPoint(3);
	if (global.GUI_GetNextCMD == 1 && !global.m4.empty())
		m_Outputs.TargetPosition[4] = global.get_dataPoint(4);
	if (global.GUI_GetNextCMD == 1 && !global.m5.empty())
		m_Outputs.TargetPosition[5] = global.get_dataPoint(5);
	if (global.GUI_GetNextCMD==1 && global.m0.empty())// || global.m0.empty())// && global.m1.empty() && global.m2.empty() && global.m3.empty() && global.m4.empty() && global.m5.empty())
		m_Outputs.GUI_GetNextCMD = 2;
	else
		m_Outputs.GUI_GetNextCMD = 1;
	return hr;
}
///</AutoGeneratedContent>

///////////////////////////////////////////////////////////////////////////////
HRESULT CMain::AddModuleToCaller()
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
VOID CMain::RemoveModuleFromCaller()
{
	m_Trace.Log(tlVerbose, FENTERA);

	if (m_spCyclicCaller)
	{
		m_spCyclicCaller->RemoveModule(m_spCyclicCaller);
	}
	m_spCyclicCaller = NULL;
	m_Trace.Log(tlVerbose, FLEAVEA);
}

///////////////////////////////////////////////////////////////////////////////
void CMain::AdsReadWriteInd
(
	AmsAddr& rAddr,
	ULONG invokeId,
	ULONG indexGroup,
	ULONG indexOffset,
	ULONG cbReadLength,
	ULONG cbWriteLength,
	PVOID pData
)
{
	m_Trace.Log(tlVerbose, FENTERA "oid=0x%08x, invokeId=%d, indexGroup=0x%08x, indexOffset=0x%08x, cbReadLength=%d, cbWriteLength=%d, pData=0x%p",
		m_objId.value, invokeId, indexGroup, indexOffset, cbReadLength, cbWriteLength, pData);

	switch (indexGroup)
	{
	case MainIndexGroup1:
		switch (indexOffset)
		{

		case MainIndexOffset1:
			m_Trace.Log(tlInfo, FNAMEA "oid=0x%08x indexGroup=0x%08x, indexOffset=0x%08x",
				m_objId.value, indexGroup, indexOffset);

			// TODO: add custom code here

			AdsReadWriteRes(rAddr, invokeId, ADSERR_NOERR, 0, NULL);
			break;

		case MainIndexOffset2:
			m_Trace.Log(tlInfo, FNAMEA "oid=0x%08x indexGroup=0x%08x, indexOffset=0x%08x",
				m_objId.value, indexGroup, indexOffset);

			// TODO: add custom code here

			AdsReadWriteRes(rAddr, invokeId, ADSERR_NOERR, 0, NULL);
			break;
		}
		break;
	case MainIndexGroup2:
		switch (indexOffset)
		{

		case MainIndexOffset1:
			m_Trace.Log(tlInfo, FNAMEA "oid=0x%08x indexGroup=0x%08x, indexOffset=0x%08x",
				m_objId.value, indexGroup, indexOffset);

			// TODO: add custom code here

			AdsReadWriteRes(rAddr, invokeId, ADSERR_NOERR, 0, NULL);
			break;

		case MainIndexOffset2:
			m_Trace.Log(tlInfo, FNAMEA "oid=0x%08x indexGroup=0x%08x, indexOffset=0x%08x",
				m_objId.value, indexGroup, indexOffset);

			// TODO: add custom code here

			AdsReadWriteRes(rAddr, invokeId, ADSERR_NOERR, 0, NULL);
			break;
		}
		break;
	default:
		__super::AdsReadWriteInd(rAddr, invokeId, indexGroup, indexOffset, cbReadLength, cbWriteLength, pData);
		break;
	}
	m_Trace.Log(tlVerbose, FLEAVEA);
}


void CMain::SubmitAdsReadReq()
{
	m_Trace.Log(tlVerbose, FENTERA);
	// m_ContextAdsPort contains the ADS port number of the task associated with context 1
	// amsAddr refers to the ADS port of this task 
	AmsAddr amsAddr(this->AmsGetNetId(), m_ContextAdsPort);

	// Request the value of a parameter from a TwinCAT module instance
	// via the task ADS port.
	// The object id of the instance is passed as index group and
	// the parameter id is passed as index offset.
	// The result is delivered by a call to AdsReadCon and can 
	// be identified by the invoke id. 
	int nRes =
		AdsReadReq
		(
			amsAddr,
			invokeIdReadByOidAndPid,
			m_objId.value,
			PID_MainCounter,
			sizeof(m_ReadByOidAndPid)
		);
	if (nRes != ADSERR_NOERR)
	{
		m_Trace.Log(tlError, FNAMEA "AdsReadReq failed with error=0x%08x(%s)", nRes, AdsGetErrorText(nRes));
	}
	else
	{
		m_Trace.Log(tlInfo, FNAMEA "AdsReadReq by oid=0x%08x and pid=0x%08x", m_objId.value, PID_MainAdsPort);
	}
	m_Trace.Log(tlVerbose, FLEAVEA);
}

///////////////////////////////////////////////////////////////////////////////
void CMain::AdsReadCon
(
	AmsAddr& rAddr,
	ULONG invokeId,
	ULONG nResult,
	ULONG cbLength,
	PVOID pData
)
{
	m_Trace.Log(tlVerbose, FENTERA "oid=0x%08x, invokeId=%d, nResult=0x%08x, cbLength=%d, pData=0x%p",
		m_objId.value, invokeId, nResult, cbLength, pData);

	if (invokeId == invokeIdReadByOidAndPid)
	{
		if (nResult != ADSERR_NOERR)
		{
			m_Trace.Log(tlWarning, FNAMEA "ReadByOidAndPid failed with error=0x%x(%s)",
				nResult, AdsGetErrorText(nResult));
		}
		else if (cbLength == sizeof(m_ReadByOidAndPid))
		{
			m_ReadByOidAndPid = *static_cast<PULONG>(pData);
			m_Trace.Log(tlInfo, FNAMEA "m_ReadByOidAndPid=0x%x", m_ReadByOidAndPid);
		}
	}
	else
	{
		__super::AdsReadWriteCon(rAddr, invokeId, nResult, cbLength, pData);
	}
	m_Trace.Log(tlVerbose, FLEAVEA);
}

