within BuildingMpc.Examples.SimulationModels;
model Case900GEOLoads
  "model to determine the geothermal load to design borefield"
  extends IDEAS.Buildings.Validation.Cases.Case900(redeclare
      BuildingMpc.Examples.SimulationModels.Envelope.Bui900TABS building);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Case900GEOLoads;
