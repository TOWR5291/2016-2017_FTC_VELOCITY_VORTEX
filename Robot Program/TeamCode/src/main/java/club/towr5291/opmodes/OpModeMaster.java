package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ianhaden on 8/02/2017.
 */

public abstract class OpModeMaster extends LinearOpMode {


    private static OpModeMaster instance = null;

    public OpModeMaster()
    {


        super();

        instance = this;

    }


}
