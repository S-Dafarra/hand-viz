service HandVisualizerCommands
{
    /**
     * Set the vertical view angle of the camera (in degrees).
     * @return true/false in case of success/failure.
     */
    bool setViewAngle(1:double angleInDeg);

    /**
     * Set the color of the hand. The values are supposed to be between 0 and 1.
     * @return true/false in case of success/failure.
     */
    bool setHandColor(1:double r, 2:double g, 3:double b);

    /**
     * Set the opacity of the hand.
     * 0 is fully transparent, 1 is fully opaque.
     * @return true/false in case of success/failure.
     */
    bool setHandOpacity(1:double opacity);

    /**
     * Set the position of the left eye with respect the head frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    bool setHeadToLeftEyeOffset(1:double x, 2:double y, 3:double z);

    /**
     * Set the position of the right eye with respect the head frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    bool setHeadToRightEyeOffset(1:double x, 2:double y, 3:double z);

    /**
     * Set the position of the left hand with respect the left frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    bool setLeftFrameToHandOffset(1:double x, 2:double y, 3:double z);

    /**
     * Set the position of the right hand with respect the right frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    bool setRightFrameToHandOffset(1:double x, 2:double y, 3:double z);

    /**
     * Set the orientation of the left hand with respect the left frame.
     * @return true/false in case of success/failure.
     */
    bool setLeftFrameToHandQuaternion(1:double w, 2:double x, 3:double y, 4:double z);

    /**
     * Set the orientation of the right hand with respect the right frame.
     * @return true/false in case of success/failure.
     */
    bool setRightFrameToHandQuaternion(1:double w, 2:double x, 3:double y, 4:double z);

    /**
     * Set the visibility of the forearms.
     * If the input boolean is true, the forearms are visible. Invisible if 0.
     * @return true/false in case of success/failure.
     */
    bool setForearmsVisibility(1:bool visible)

    /**
     * Prints the settings
     * @return A string that can be copied in a configuration file,
     *         including the modifications set from RPC
     */
    string printSettings();
}
