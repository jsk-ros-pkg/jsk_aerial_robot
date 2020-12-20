add_rostest(dragon_jacobian.test ARGS headless:=true)
add_rostest(dragon_control.test ARGS headless:=true) # old control method: hydrus-like LQI mode
add_rostest(dragon_control.test ARGS headless:=true full_vectoring_mode:=true) # new control method
