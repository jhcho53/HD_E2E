
"use strict";

let ReplayInfo = require('./ReplayInfo.js');
let TrafficLight = require('./TrafficLight.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let CollisionData = require('./CollisionData.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let Transforms = require('./Transforms.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let IntersectionControl = require('./IntersectionControl.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let SensorPosControl = require('./SensorPosControl.js');
let PREvent = require('./PREvent.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let CtrlCmd = require('./CtrlCmd.js');
let SaveSensorData = require('./SaveSensorData.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let DillyCmd = require('./DillyCmd.js');
let PRStatus = require('./PRStatus.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let WheelControl = require('./WheelControl.js');
let Obstacles = require('./Obstacles.js');
let Conveyor = require('./Conveyor.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let RobotState = require('./RobotState.js');
let RadarDetections = require('./RadarDetections.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let EventInfo = require('./EventInfo.js');
let ShipState = require('./ShipState.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let WaitForTick = require('./WaitForTick.js');
let VehicleSpec = require('./VehicleSpec.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let RobotOutput = require('./RobotOutput.js');
let Lamps = require('./Lamps.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let VelocityCmd = require('./VelocityCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let IntscnTL = require('./IntscnTL.js');
let VehicleCollision = require('./VehicleCollision.js');
let ERP42Info = require('./ERP42Info.js');
let GVStateCmd = require('./GVStateCmd.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let CMDConveyor = require('./CMDConveyor.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let ObjectStatus = require('./ObjectStatus.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let ExternalForce = require('./ExternalForce.js');
let Obstacle = require('./Obstacle.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let SVADC = require('./SVADC.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let RadarDetection = require('./RadarDetection.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let TOF = require('./TOF.js');
let GPSMessage = require('./GPSMessage.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let MapSpec = require('./MapSpec.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let GhostMessage = require('./GhostMessage.js');

module.exports = {
  ReplayInfo: ReplayInfo,
  TrafficLight: TrafficLight,
  MoraiSimProcStatus: MoraiSimProcStatus,
  CollisionData: CollisionData,
  MoraiSimProcHandle: MoraiSimProcHandle,
  GeoVector3Message: GeoVector3Message,
  Transforms: Transforms,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  IntersectionControl: IntersectionControl,
  FaultInjection_Controller: FaultInjection_Controller,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  SensorPosControl: SensorPosControl,
  PREvent: PREvent,
  MapSpecIndex: MapSpecIndex,
  CtrlCmd: CtrlCmd,
  SaveSensorData: SaveSensorData,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  MoraiTLInfo: MoraiTLInfo,
  DillyCmd: DillyCmd,
  PRStatus: PRStatus,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  FaultInjection_Tire: FaultInjection_Tire,
  SyncModeRemoveObject: SyncModeRemoveObject,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  WheelControl: WheelControl,
  Obstacles: Obstacles,
  Conveyor: Conveyor,
  FaultStatusInfo: FaultStatusInfo,
  NpcGhostCmd: NpcGhostCmd,
  ShipCtrlCmd: ShipCtrlCmd,
  MultiPlayEventRequest: MultiPlayEventRequest,
  RobotState: RobotState,
  RadarDetections: RadarDetections,
  ObjectStatusList: ObjectStatusList,
  EventInfo: EventInfo,
  ShipState: ShipState,
  DillyCmdResponse: DillyCmdResponse,
  WaitForTick: WaitForTick,
  VehicleSpec: VehicleSpec,
  EgoVehicleStatus: EgoVehicleStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  RobotOutput: RobotOutput,
  Lamps: Lamps,
  MultiPlayEventResponse: MultiPlayEventResponse,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  VelocityCmd: VelocityCmd,
  NpcGhostInfo: NpcGhostInfo,
  PRCtrlCmd: PRCtrlCmd,
  IntscnTL: IntscnTL,
  VehicleCollision: VehicleCollision,
  ERP42Info: ERP42Info,
  GVStateCmd: GVStateCmd,
  ScenarioLoad: ScenarioLoad,
  IntersectionStatus: IntersectionStatus,
  CMDConveyor: CMDConveyor,
  ManipulatorControl: ManipulatorControl,
  MultiEgoSetting: MultiEgoSetting,
  ObjectStatusExtended: ObjectStatusExtended,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  ObjectStatus: ObjectStatus,
  DdCtrlCmd: DdCtrlCmd,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  SyncModeCmd: SyncModeCmd,
  MoraiTLIndex: MoraiTLIndex,
  SyncModeCmdResponse: SyncModeCmdResponse,
  GetTrafficLightStatus: GetTrafficLightStatus,
  ExternalForce: ExternalForce,
  Obstacle: Obstacle,
  SyncModeInfo: SyncModeInfo,
  SyncModeResultResponse: SyncModeResultResponse,
  SVADC: SVADC,
  SyncModeAddObject: SyncModeAddObject,
  MoraiSrvResponse: MoraiSrvResponse,
  RadarDetection: RadarDetection,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  SetTrafficLight: SetTrafficLight,
  TOF: TOF,
  GPSMessage: GPSMessage,
  VehicleCollisionData: VehicleCollisionData,
  MapSpec: MapSpec,
  FaultInjection_Response: FaultInjection_Response,
  GVDirectCmd: GVDirectCmd,
  SyncModeSetGear: SyncModeSetGear,
  VehicleSpecIndex: VehicleSpecIndex,
  SkateboardStatus: SkateboardStatus,
  WoowaDillyStatus: WoowaDillyStatus,
  WaitForTickResponse: WaitForTickResponse,
  FaultInjection_Sensor: FaultInjection_Sensor,
  GhostMessage: GhostMessage,
};
