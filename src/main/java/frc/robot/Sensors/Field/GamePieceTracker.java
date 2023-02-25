package frc.robot.Sensors.Field;

import java.util.List;
import java.util.Map;

public class GamePieceTracker {
    
    private static GamePieceTracker instance;

    private static List<Map.Entry<GamePiece, Boolean>> emptyRow = List.of(
        Map.entry(GamePiece.cone, false),
        Map.entry(GamePiece.cube, false),
        Map.entry(GamePiece.cone, false),
        Map.entry(GamePiece.cone, false),
        Map.entry(GamePiece.cube, false),
        Map.entry(GamePiece.cone, false),
        Map.entry(GamePiece.cone, false),
        Map.entry(GamePiece.cube, false),
        Map.entry(GamePiece.cone, false)
    );

    private List<List<Map.Entry<GamePiece, Boolean>>> scoring = List.of(
        emptyRow,
        emptyRow,
        emptyRow
    );

    private enum GamePiece {
        cube,
        cone;
    }

    public static GamePieceTracker getInstance() {
        if (instance == null) instance = new GamePieceTracker();
        return instance;
    }

    /**
     * Sets the game piece to scored/unscored
     * @param height Scoring Level (0 - Lowest -> 2 - Heighest)
     * @param row Scoring Row (0 - Closest to Loading Station -> 8 - Farthest from Loading Station)
     * @param b
     */
    public void setPiece(int height, int row, boolean b) {
        scoring.get(height).get(row).setValue(b);
    }

}
