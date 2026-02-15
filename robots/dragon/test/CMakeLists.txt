add_rostest(dragon_jacobian.test ARGS headless:=true)
add_rostest(dragon_control.test ARGS headless:=true new_model:=false) # old control method: hydrus-like LQI mode with model v1.0
add_rostest(dragon_control.test ARGS headless:=true full_vectoring_mode:=true) # new control method with model v1.5
