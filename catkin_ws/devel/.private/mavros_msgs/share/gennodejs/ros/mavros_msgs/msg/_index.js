
"use strict";

let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let SysStatus = require('./SysStatus.js');
let Thrust = require('./Thrust.js');
let GPSINPUT = require('./GPSINPUT.js');
let LandingTarget = require('./LandingTarget.js');
let HilGPS = require('./HilGPS.js');
let HilControls = require('./HilControls.js');
let WaypointList = require('./WaypointList.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let Mavlink = require('./Mavlink.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ManualControl = require('./ManualControl.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let LogData = require('./LogData.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let WaypointReached = require('./WaypointReached.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let RTCM = require('./RTCM.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let ParamValue = require('./ParamValue.js');
let PositionTarget = require('./PositionTarget.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let LogEntry = require('./LogEntry.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let StatusText = require('./StatusText.js');
let MountControl = require('./MountControl.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let Altitude = require('./Altitude.js');
let HomePosition = require('./HomePosition.js');
let ExtendedState = require('./ExtendedState.js');
let TerrainReport = require('./TerrainReport.js');
let CommandCode = require('./CommandCode.js');
let ActuatorControl = require('./ActuatorControl.js');
let HilSensor = require('./HilSensor.js');
let VFR_HUD = require('./VFR_HUD.js');
let RadioStatus = require('./RadioStatus.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CellularStatus = require('./CellularStatus.js');
let ESCStatus = require('./ESCStatus.js');
let Tunnel = require('./Tunnel.js');
let VehicleInfo = require('./VehicleInfo.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let Vibration = require('./Vibration.js');
let FileEntry = require('./FileEntry.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let DebugValue = require('./DebugValue.js');
let ESCInfo = require('./ESCInfo.js');
let BatteryStatus = require('./BatteryStatus.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let State = require('./State.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let GPSRAW = require('./GPSRAW.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let RTKBaseline = require('./RTKBaseline.js');
let Trajectory = require('./Trajectory.js');
let RCIn = require('./RCIn.js');
let Waypoint = require('./Waypoint.js');
let RCOut = require('./RCOut.js');
let Param = require('./Param.js');
let GPSRTK = require('./GPSRTK.js');

module.exports = {
  GlobalPositionTarget: GlobalPositionTarget,
  SysStatus: SysStatus,
  Thrust: Thrust,
  GPSINPUT: GPSINPUT,
  LandingTarget: LandingTarget,
  HilGPS: HilGPS,
  HilControls: HilControls,
  WaypointList: WaypointList,
  EstimatorStatus: EstimatorStatus,
  Mavlink: Mavlink,
  OverrideRCIn: OverrideRCIn,
  ManualControl: ManualControl,
  WheelOdomStamped: WheelOdomStamped,
  LogData: LogData,
  OpticalFlowRad: OpticalFlowRad,
  WaypointReached: WaypointReached,
  ESCTelemetry: ESCTelemetry,
  RTCM: RTCM,
  HilActuatorControls: HilActuatorControls,
  ParamValue: ParamValue,
  PositionTarget: PositionTarget,
  CompanionProcessStatus: CompanionProcessStatus,
  LogEntry: LogEntry,
  MagnetometerReporter: MagnetometerReporter,
  ESCInfoItem: ESCInfoItem,
  StatusText: StatusText,
  MountControl: MountControl,
  CamIMUStamp: CamIMUStamp,
  PlayTuneV2: PlayTuneV2,
  ESCTelemetryItem: ESCTelemetryItem,
  Altitude: Altitude,
  HomePosition: HomePosition,
  ExtendedState: ExtendedState,
  TerrainReport: TerrainReport,
  CommandCode: CommandCode,
  ActuatorControl: ActuatorControl,
  HilSensor: HilSensor,
  VFR_HUD: VFR_HUD,
  RadioStatus: RadioStatus,
  AttitudeTarget: AttitudeTarget,
  ADSBVehicle: ADSBVehicle,
  CellularStatus: CellularStatus,
  ESCStatus: ESCStatus,
  Tunnel: Tunnel,
  VehicleInfo: VehicleInfo,
  TimesyncStatus: TimesyncStatus,
  Vibration: Vibration,
  FileEntry: FileEntry,
  NavControllerOutput: NavControllerOutput,
  OnboardComputerStatus: OnboardComputerStatus,
  DebugValue: DebugValue,
  ESCInfo: ESCInfo,
  BatteryStatus: BatteryStatus,
  ESCStatusItem: ESCStatusItem,
  State: State,
  HilStateQuaternion: HilStateQuaternion,
  GPSRAW: GPSRAW,
  CameraImageCaptured: CameraImageCaptured,
  RTKBaseline: RTKBaseline,
  Trajectory: Trajectory,
  RCIn: RCIn,
  Waypoint: Waypoint,
  RCOut: RCOut,
  Param: Param,
  GPSRTK: GPSRTK,
};
