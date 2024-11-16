package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.IOException;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

@TeleOp(name = "\uD83D\uDEDC GeckoServer", group = "Gecko")
public class GeckoServer extends LinearOpMode {
    DcMotorEx intake;
    @Override
    public void runOpMode() {
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
      intake = drive.Intake;

      try {
        new HttpServer(3000);
          telemetry.addData("Status", "Server started on port 3000");
          telemetry.update();
      } catch (IOException e) {
          telemetry.addData("Error", "Failed to start server: " + e.getMessage());
          telemetry.update();
      }

      waitForStart();
    }

    class HttpServer extends NanoHTTPD {
      public HttpServer(int port) throws IOException {
        super(port);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
      }

      @Override
      public Response serve(IHTTPSession session) {
        if (Method.OPTIONS.equals(session.getMethod())) {
          Response res = newFixedLengthResponse(Response.Status.OK, MIME_PLAINTEXT, "");
          res.addHeader("Access-Control-Allow-Origin", "*");
          res.addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
          res.addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
          return res;
        }

        String path = session.getUri();
        Map<String, String> queryParams = session.getParms();

        switch (path) {
          case "/toggle_intake":
            boolean isOn = Boolean.parseBoolean(queryParams.get("state"));
            intake.setPower(isOn ? -0.5 : 0);

            break;
        }

        Response response = newFixedLengthResponse(Response.Status.OK, MIME_PLAINTEXT, "OK");
        response.addHeader("Access-Control-Allow-Origin", "*");
        response.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        response.addHeader("Access-Control-Allow-Headers", "Content-Type");
        return response;
      }
    }
}