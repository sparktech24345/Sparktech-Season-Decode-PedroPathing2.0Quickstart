package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode.publicTelemetry;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.json.JsonReadFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;

import java.util.Map;

@com.acmerobotics.dashboard.config.Config
public class Config {

    public int    teamPipeline = 0;

    public double farZoneCameraAdder = 0;
    public double autoZoneAdderFar = 0;

    public double autoVelAdder = 0;
    public double rotationForInitCloseZone = 0;
    public double rotationForInitSmallTriangle = 0;
    public double targetX = 0;
    public double targetY = 0;
    public double targetXAutoClose = 0;
    public double targetYAutoClose = 0;
    public double targetXCenter = 0;
    public double targetYCenter = 0;
    public double targetXRightPanel = 0;
    public double targetYRightPanel = 0;
    public double targetXLeftPanel = 0;
    public double targetYLeftPanel = 0;
    public double usedTargetX = 0;
    public double usedTargetY = 0;
    public double targetForCameraX = 0;
    public double targetForCameraY = 0;
    public double targetForClassifierX = 0;
    public double targetForClassifierXNumber2 = 0;
    public double targetForClassifierY = 0;
    public double targetForClassifierYNumber2 = 0;
    public double targetForFirstClassifierScan = 0;
    public double hpResetX = 0;
    public double hpResetY = 0;
    public double hpResetDeg = 0;
    public double classifierResetX = 0;
    public double classifierResetY = 0;
    public double classifierResetDeg = 0;

    public TeamColor currentTeamColor = TeamColor.TeamNotSet;

    private final static String json = """
    {
    "blue": {
        "teamPipeline": 0,

        "farZoneCameraAdder": 0,
        "autoZoneAdderFar": 0,

        "targetForCameraX": 12,
        "targetForCameraY": 50,

        "targetForClassifierX": 75,
        "targetForClassifierY": 47,

        "targetForClassifierXNumber2": 85,
        "targetForClassifierYNumber2": 50,

        "targetForFirstClassifierScan": 0,

        "autoVelAdder": 0,

        "rotationForInitCloseZone": 250,
        "rotationForInitSmallTriangle": 90,

        "targetX": 130,
        "targetY": 53,

        "targetXAutoClose": 127,
        "targetYAutoClose": 49,

        "targetXCenter": 128, // 130
        "targetYCenter": 53,

        "targetXRightPanel": 127,
        "targetYRightPanel": 48,

        "targetXLeftPanel": 129,
        "targetYLeftPanel": 55,

        "usedTargetX": 131,
        "usedTargetY": 40,

        "currentTeamColor": "Blue",

        "hpResetX": 0.5,
        "hpResetY": -78.9,
        "hpResetDeg": 90,

        "classifierResetX": 122,
        "classifierResetY": 30.5,
        "classifierResetDeg": -135,
    },
    "red": {
        "teamPipeline": 1,

        "farZoneCameraAdder": 0,
        "autoZoneAdderFar": 0,


        "targetForCameraX": 12,
        "targetForCameraY": -50,


        "targetForClassifierX": 75,
        "targetForClassifierY": -47,

        "targetForFirstClassifierScan": 345,

        "targetForClassifierXNumber2": 85,
        "targetForClassifierYNumber2": -50,

        "autoVelAdder": 0,

        "rotationForInitCloseZone": 100,
        "rotationForInitSmallTriangle": 270,

        "targetX": 130,
        "targetY": -53,

        "targetXAutoClose": 127,
        "targetYAutoClose": -43,

        "targetXCenter": 128, // 130
        "targetYCenter": -53,

        "targetXRightPanel": 127,
        "targetYRightPanel": -43,

        "targetXLeftPanel": 125,
        "targetYLeftPanel": -55,

        "usedTargetX": 131,
        "usedTargetY": -40,

        "currentTeamColor": "Red",


        "hpResetX": 0.5,
        "hpResetY": 78.9, // with plus
        "hpResetDeg": -90,

        "classifierResetX": 122,
        "classifierResetY": -30.5,
        "classifierResetDeg": 135,
    },
    }
    """;

    public static Config getConfig(String cfg_name) {
        ObjectMapper mapper = new ObjectMapper();
        mapper.configure(JsonReadFeature.ALLOW_JAVA_COMMENTS.mappedFeature(), true);
        try {
            Map<String, Config> configs = mapper.readValue(json, mapper.getTypeFactory().constructMapType(Map.class, String.class, Config.class));
            return configs.get(cfg_name);
        } catch (JsonProcessingException e) {
            publicTelemetry.addData("Could not load config:", e.getMessage());
            publicTelemetry.update();
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e2) {
                publicTelemetry.addData("Bro can't even sleep :(", e2.getMessage());
            }
        }
        return null;
    }
}