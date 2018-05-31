within BuildingMpc.Examples.Data;
record HeavyFloorTABS "BESTEST heavy floor with TABS"
  extends IDEAS.Buildings.Data.Interfaces.Construction(
    incLastLay = IDEAS.Types.Tilt.Floor,
    locGain = {2},
    final mats={
      IDEAS.Buildings.Validation.Data.Insulation.InsulationFloor(d=1.007),
      IDEAS.Buildings.Validation.Data.Materials.ConcreteSlab(d=0.08-0.0565),
      IDEAS.Buildings.Validation.Data.Materials.ConcreteSlab(d=0.0565)});
end HeavyFloorTABS;
