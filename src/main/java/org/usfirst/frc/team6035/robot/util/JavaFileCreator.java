package org.usfirst.frc.team6035.robot.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.List;

import static java.nio.file.Files.createDirectory;
import static java.nio.file.Files.createFile;

public class JavaFileCreator {

    public static void createClass(String name, Track track) {
        String content = buildContent(name, track);
        writeFile(name, content);
    }

    private static String buildContent(String name, Track track) {
        StringBuilder builder = new StringBuilder() ;

        builder.append("package org.usfirst.frc.team6035.robot\n\n")

                .append("import org.usfirst.frc.team6035.robot.*;\n\n")

                .append("public class ").append(name).append(" extends AutoDirection {\n\n");

        appendSpeedContent("leftMotorSpeeds", track.leftSpeeds(), builder);
        appendSpeedContent("rightMotorSpeeds", track.rightSpeeds(), builder);

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

    private static void appendSpeedContent(String variableName, List<Double> speeds, StringBuilder builder) {
        builder.append("  private double[] ").append(variableName).append(" = {\n").append("    ");

        Sep sep = new Sep(", ");
        speeds.forEach(s -> {
            String sf = String.format("%.2f", s);
            builder.append(sep.sep()).append(sf);
        });

        builder.append("\n")
                .append("  };\n\n");
    }

    private static class Sep {
        private String sep;
        private String nextSep = "";

        public Sep(String sep) {
            this.sep = sep;
        }

        public String sep() {
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
