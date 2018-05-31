within BuildingMpc.Examples.DesignOpt.DecisionVariablesRecords.Templates;
record IntegerVariable "Boolean decision variable"
  extends Templates.DiscreteDecisionVariable(
    final i=y);
  final parameter Integer val = vals[y] "Design variable value";
  parameter Integer y(start=1) "Actual decision variable from stochastic algorithm";
  parameter Integer[:] vals "Design variable values";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end IntegerVariable;
