import numpy as np
import matplotlib.pyplot as plt


def gen_tray_arado_horizontal(dt):

    speed_tray_rec = 1.9
    speed_tray_curv = 1.3

    field_width = 40.0
    pos_start = [0., 0.]
    vertical_step = 10.0
    time_array = np.arange(0, np.pi * vertical_step / (2. * speed_tray_curv), dt)

    n_reps = 1  # numero de veces que se pone la referencia correspondiente al punto inicial y final de cada trayectoria

    porcent_tray_lin_slow = .25
    porcent_speed = 1.
    x_rect_0 = np.linspace(pos_start[0], pos_start[0] + field_width * porcent_tray_lin_slow,
                           int(field_width * porcent_tray_lin_slow / float(speed_tray_rec * porcent_speed * dt)),
                           endpoint=False)
    x_rect_1 = np.linspace(pos_start[0] + field_width * porcent_tray_lin_slow,
                           pos_start[0] + field_width * (1. - porcent_tray_lin_slow),
                           int(field_width * (1 - 2. * porcent_tray_lin_slow) / float(speed_tray_rec * dt)),
                           endpoint=False)
    x_rect_2 = np.linspace(pos_start[0] + field_width * (1. - porcent_tray_lin_slow), pos_start[0] + field_width,
                           int(field_width * porcent_tray_lin_slow / float(speed_tray_rec * porcent_speed * dt)),
                           endpoint=False)
    x_rect = np.concatenate([x_rect_0, x_rect_1, x_rect_2, [x_rect_2[-1]] * (n_reps - 1)])
    y_rect = np.ones_like(x_rect) * pos_start[1]
    cent_circ = [pos_start[0] + field_width, pos_start[1] + vertical_step / 2.]
    x_circ = cent_circ[0] + np.sin(time_array * 2. * speed_tray_curv / float(vertical_step)) * vertical_step / 2.
    y_circ = cent_circ[1] - np.cos(time_array * 2. * speed_tray_curv / float(vertical_step)) * vertical_step / 2.
    x_tramo = np.concatenate([x_rect, x_circ, [x_circ[-1]] * (n_reps - 1)])
    y_tramo = np.concatenate([y_rect, y_circ, [y_circ[-1]] * (n_reps - 1)])
    x_ida_y_vuelta = np.array([x_tramo, field_width - x_tramo]).flatten()
    y_ida_y_vuelta = np.array([y_tramo, y_tramo + vertical_step]).flatten()
    x = np.copy(x_ida_y_vuelta)
    y = np.copy(y_ida_y_vuelta)

    n_lines = 5
    assert (n_lines % 2 == 1)
    for i in range(int(n_lines / 2. - .5) - 1):
        x = np.concatenate((x, x_ida_y_vuelta))
        y = np.concatenate((y, y_ida_y_vuelta + vertical_step * 2 * (i + 1)))  # , y_rect + vertical_step*4))
    x = np.concatenate((x, x_rect))
    y = np.concatenate((y, y_rect + vertical_step * (n_lines - 1)))

    return np.array([x, y]).T
