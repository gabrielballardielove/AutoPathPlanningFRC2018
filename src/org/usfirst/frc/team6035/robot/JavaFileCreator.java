package org.usfirst.frc.team6035.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static java.nio.file.Files.createDirectory;

public class JavaFileCreator {

    public static void createClass(String name, double[] LeftSpeeds, double [] RightSpeeds) {
        String content = buildContent(name, LeftSpeeds, RightSpeeds);
        writeFile(name, content);
    }

    private static String buildContent(String name, double[] LeftSpeeds, double[] RightSpeeds) {
        StringBuilder builder = new StringBuilder() ;

        builder.append("package org.usfirst.frc.team6035.robot.auto;\n\n")
                .append("public class ").append(name).append(" extends AutoDirection {\n\n");

        appendSpeedContent("leftMotorSpeeds", LeftSpeeds, builder);
        appendSpeedContent("rightMotorSpeeds", RightSpeeds, builder);

        builder.append("  public int nSteps() {\n")
                .append("    return leftMotorSpeeds.length;\n")
                .append("  }\n\n")

                .append("  public double leftSpeed(int i) {\n")
                .append("    return leftMotorSpeeds[i];\n")
                .append("  }\n\n")

                .append("  public double rightSpeed(int i) {\n")
                .append("    return rightMotorSpeeds[i];\n")
                .append("  }\n\n");

        builder.append("}\n");

        return builder.toString();
    }

    private static void appendSpeedContent(String variableName, double[] speeds, StringBuilder builder) {
        builder.append("  private double[] ").append(variableName).append(" = {\n").append("    ");

        Sep sep = new Sep();
       for (int i = 0; i < speeds.length; i++) {
    	   String sf = String.format("%.2f", speeds[i]);
    	   builder.append(sep.sep()).append(sf);
       }

        builder.append("\n")
                .append("  };\n\n");
    }

    private static class Sep {
        private final String sep;
        private String nextSep = "";

        Sep() {
            this.sep = ", ";
        }

        String sep() {
            String s = nextSep;
            nextSep = sep;
            return s;
        }
    }

    private static void writeFile(String name, String content) {
        try {
            writeFileEx(name, content);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void writeFileEx(String name, String content) throws IOException {
        final Path out = Paths.get("out");

        if (Files.notExists(out)) {
            createDirectory(out);
        }

        final Path file = out.resolve(name + ".java");
        Files.write(file, content.getBytes());
    }
}