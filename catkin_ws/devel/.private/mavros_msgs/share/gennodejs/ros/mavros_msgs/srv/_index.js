
"use strict";

let CommandLong = require('./CommandLong.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let ParamPull = require('./ParamPull.js')
let CommandHome = require('./CommandHome.js')
let WaypointClear = require('./WaypointClear.js')
let FileClose = require('./FileClose.js')
let FileRead = require('./FileRead.js')
let MessageInterval = require('./MessageInterval.js')
let FileOpen = require('./FileOpen.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let ParamPush = require('./ParamPush.js')
let WaypointPush = require('./WaypointPush.js')
let CommandAck = require('./CommandAck.js')
let FileTruncate = require('./FileTruncate.js')
let ParamSet = require('./ParamSet.js')
let LogRequestData = require('./LogRequestData.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamGet = require('./ParamGet.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileChecksum = require('./FileChecksum.js')
let CommandInt = require('./CommandInt.js')
let CommandTOL = require('./CommandTOL.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let LogRequestList = require('./LogRequestList.js')
let MountConfigure = require('./MountConfigure.js')
let FileRename = require('./FileRename.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileRemove = require('./FileRemove.js')
let WaypointPull = require('./WaypointPull.js')
let CommandBool = require('./CommandBool.js')
let FileList = require('./FileList.js')
let StreamRate = require('./StreamRate.js')
let FileWrite = require('./FileWrite.js')
let SetMode = require('./SetMode.js')
let FileMakeDir = require('./FileMakeDir.js')

module.exports = {
  CommandLong: CommandLong,
  WaypointSetCurrent: WaypointSetCurrent,
  ParamPull: ParamPull,
  CommandHome: CommandHome,
  WaypointClear: WaypointClear,
  FileClose: FileClose,
  FileRead: FileRead,
  MessageInterval: MessageInterval,
  FileOpen: FileOpen,
  CommandVtolTransition: CommandVtolTransition,
  LogRequestEnd: LogRequestEnd,
  ParamPush: ParamPush,
  WaypointPush: WaypointPush,
  CommandAck: CommandAck,
  FileTruncate: FileTruncate,
  ParamSet: ParamSet,
  LogRequestData: LogRequestData,
  FileRemoveDir: FileRemoveDir,
  ParamGet: ParamGet,
  VehicleInfoGet: VehicleInfoGet,
  SetMavFrame: SetMavFrame,
  FileChecksum: FileChecksum,
  CommandInt: CommandInt,
  CommandTOL: CommandTOL,
  CommandTriggerInterval: CommandTriggerInterval,
  LogRequestList: LogRequestList,
  MountConfigure: MountConfigure,
  FileRename: FileRename,
  CommandTriggerControl: CommandTriggerControl,
  FileRemove: FileRemove,
  WaypointPull: WaypointPull,
  CommandBool: CommandBool,
  FileList: FileList,
  StreamRate: StreamRate,
  FileWrite: FileWrite,
  SetMode: SetMode,
  FileMakeDir: FileMakeDir,
};
