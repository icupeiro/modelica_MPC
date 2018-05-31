within BuildingMpc.Examples.DesignOpt.DecisionVariablesRecords.Templates;
record BooleanVariable "Boolean decision variable"
  extends DiscreteDecisionVariable(
    final numOpts=2,
    i=y);
  parameter Integer y;
  final parameter Boolean val = y==2 "Decision variable value";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BooleanVariable;
