"""cuRobo 기반 Move-and-Look 서비스 entry point."""

from isaac_control_core.services import run_move_and_look
from curobo_control.core import CuroboController


def main(args=None):
    run_move_and_look(
        CuroboController,
        args=args,
        curobo_config_path="/root/ws/src/curobo_control/config/piper.yml",
    )


if __name__ == "__main__":
    main()
