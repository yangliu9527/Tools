import numpy as np
import argparse


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2

    return np.array([x, y, z, w])


def main():

    parser = argparse.ArgumentParser(
        description="Fix pose file by applying Rx(pi) to Twc poses"
    )

    parser.add_argument(
        "-i", "--input",
        required=True,
        help="input pose file"
    )

    parser.add_argument(
        "-o", "--output",
        required=True,
        help="output pose file"
    )

    args = parser.parse_args()

    input_file = args.input
    output_file = args.output

    # Rx(pi) quaternion
    q_rx = np.array([1.0, 0.0, 0.0, 0.0])

    # Rx(pi) matrix
    Rx = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    with open(input_file) as f, open(output_file, "w") as fo:

        for line in f:

            line = line.strip()
            if not line:
                continue

            tokens = line.split()

            ts = tokens[0]  # 保持原始字符串

            t = np.array(list(map(float, tokens[1:4])))
            q = np.array(list(map(float, tokens[4:8])))

            # transform translation
            t_new = Rx @ t

            # transform rotation
            q_new = quat_mul(q_rx, q)

            fo.write(
                f"{ts} "
                f"{t_new[0]} {t_new[1]} {t_new[2]} "
                f"{q_new[0]} {q_new[1]} {q_new[2]} {q_new[3]}\n"
            )

    print(f"done: {output_file}")


if __name__ == "__main__":
    main()