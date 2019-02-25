package frc.robot.lib.vision;

import edu.wpi.first.wpilibj.Timer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import java.util.ArrayList;

public class MkPixy {

    private static final int blockSignature = 1;
    private static ArrayList<Block> blocks = null;
    private static Pixy2 pixy = null;
    private static MkPixyTarget mTarget = new MkPixyTarget(0,0,false, 0);
    private static int PixyResult = 0;


    public MkPixy() {
        pixy = Pixy2.createInstance(LinkType.SPI);
        PixyResult = pixy.init();
    }

    public void pixyUpdate() {
        if (PixyResult == 0) {
            int updateStatus = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 10);
            if (updateStatus > 0) {
                blocks = pixy.getCCC().getBlocks();
                Block largestBlock = null;
                for (Block block : blocks) {
                    if (block.getSignature() == blockSignature) {
                        if (largestBlock == null) {
                            largestBlock = block;
                        } else if (block.getWidth() * block.getHeight() > largestBlock.getWidth() * largestBlock.getHeight()) {
                            largestBlock = block;
                        }
                    }
                }
                if (largestBlock != null) {
                    mTarget = new MkPixyTarget(largestBlock.getX(), largestBlock.getWidth() * largestBlock.getHeight(), true, Timer.getFPGATimestamp());
                } else {
                    mTarget = new MkPixyTarget(0, 0, false, Timer.getFPGATimestamp());
                }
            }
        }
    }

    public MkPixyTarget getLatestTarget() {
        return mTarget;
    }

}
