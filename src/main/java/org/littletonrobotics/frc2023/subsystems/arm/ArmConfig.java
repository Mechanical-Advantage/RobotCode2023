// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.io.File;
import java.io.IOException;
import java.util.Map;

/** Represents all of the arm config data shared between the robot code and solver. */
public record ArmConfig(
    Translation2d origin,
    JointConfig shoulder,
    JointConfig elbow,
    JointConfig wrist,
    SolverConfig solver,
    Map<String, Constraint> constraints) {

  /** Physics constants for a single joint. */
  public static record JointConfig(
      double mass,
      double length,
      double moi,
      double cgRadius,
      double minAngle,
      double maxAngle,
      double reduction,
      DCMotor motor) {}

  /** Config fields for solver. */
  public static record SolverConfig(int interiorPoints, double maxVoltage) {}

  /** Arbitrary solver constraint. */
  public static record Constraint(String type, double[] args) {}

  /** Converts double array to Translation2d instance. */
  private static class Translation2dDeserializer extends StdDeserializer<Translation2d> {
    public Translation2dDeserializer() {
      this(null);
    }

    public Translation2dDeserializer(Class<?> vc) {
      super(vc);
    }

    @Override
    public Translation2d deserialize(JsonParser jp, DeserializationContext ctxt)
        throws IOException, JsonProcessingException {
      ArrayNode node = jp.getCodec().readTree(jp);
      double x = (Double) node.get(0).numberValue();
      double y = (Double) node.get(1).numberValue();
      return new Translation2d(x, y);
    }
  }

  /** Converts motor type, count, and reduction to DCMotor instance. */
  private static class DCMotorDeserializer extends StdDeserializer<DCMotor> {
    public DCMotorDeserializer() {
      this(null);
    }

    public DCMotorDeserializer(Class<?> vc) {
      super(vc);
    }

    @Override
    public DCMotor deserialize(JsonParser jp, DeserializationContext ctxt)
        throws IOException, JsonProcessingException {
      JsonNode node = jp.getCodec().readTree(jp);
      String type = node.get("type").asText();
      int count = (Integer) ((IntNode) node.get("count")).numberValue();
      double reduction = (Double) ((DoubleNode) node.get("reduction")).numberValue();

      switch (type) {
        case "neo":
          return DCMotor.getNEO(count).withReduction(reduction);
        case "neo550":
          return DCMotor.getNeo550(count).withReduction(reduction);
        default:
          return null;
      }
    }
  }

  /** Generates a config instance by reading from a JSON file. */
  public static ArmConfig loadJson(File source) {
    // Set up object mapper
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    SimpleModule module = new SimpleModule();
    module.addDeserializer(Translation2d.class, new Translation2dDeserializer());
    module.addDeserializer(DCMotor.class, new DCMotorDeserializer());
    mapper.registerModule(module);

    // Read config data
    ArmConfig config;
    try {
      config = mapper.readValue(source, ArmConfig.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse arm config JSON");
    }

    // Return result
    return config;
  }
}
