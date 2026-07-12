from approachtwo.model import allocated_motor_speeds, create_A, pseudoinverse


def allocate(u, motors_active=None):
    return allocated_motor_speeds(u, motors_active)


__all__ = ["allocate", "allocated_motor_speeds", "create_A", "pseudoinverse"]
