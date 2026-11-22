"""Render a functional wiring diagram, not a physical header orientation map."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle


def main():
    out = Path(__file__).resolve().parents[1] / "images"
    plt.rcParams.update({"font.family": "DejaVu Sans", "svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(12, 9.5), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set(xlim=(0, 1200), ylim=(0, 950))
    ax.axis("off")
    ink, muted, blue, red = "#17212b", "#536171", "#1c5f91", "#a72e2e"

    def txt(x, y, text, size=12, color=ink, **kw):
        ax.text(x, y, text, fontsize=size, color=color, va="center", **kw)

    txt(35, 918, "ESP32-S3 Super Mini ↔ LSM6DSV", 20, weight="bold")
    txt(35, 884, "Thiết kế SPI 4 dây + INT1 • Xe dùng MKS Servo42C • 21/11/2026", 11, muted)
    txt(35, 854, "Đấu theo TÊN CHÂN. Hình này không mô tả thứ tự lỗ hàn trên module.", 10, muted)

    for x, title, subtitle in ((35, "ESP32-S3", "Các GPIO hiện có"), (860, "MODULE LSM6DSV", "Nhãn chân trong ảnh")):
        ax.add_patch(Rectangle((x, 376), 305, 435, facecolor="#f5f7f9", edgecolor="#c5cdd5", lw=1))
        txt(x + 20, 783, title, 14, weight="bold")
        txt(x + 20, 751, subtitle, 10, muted)

    rows = [
        ("3V3", "3V3", "Nguồn 3,3 V", red, False),
        ("GND", "GND", "Mass chung", muted, False),
        ("GPIO11", "SCL", "SCK · clock", blue, False),
        ("GPIO10", "SDA", "MOSI · ESP → IMU", blue, False),
        ("GPIO9 · INPUT", "SA0 / SDO", "MISO · IMU → ESP", blue, True),
        ("GPIO8", "CS", "Chọn chip · active LOW", blue, False),
        ("GPIO7 · INPUT", "INT1", "Data-ready · IMU → ESP", blue, True),
    ]
    for i, (left, right, label, color, reverse) in enumerate(rows):
        y = 704 - i * 48
        txt(55, y, left, 13, weight="bold")
        txt(880, y, right, 13, weight="bold")
        start, end = ((857, y), (344, y)) if reverse else ((344, y), (857, y))
        ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=13, color=color, lw=1.7))
        txt(600, y + 15, label, 10, color, ha="center", bbox={"facecolor": "white", "edgecolor": "none", "pad": 1})

    txt(35, 342, "Các chân phụ và phần kéo mức", 13, weight="bold")
    notes = [
        "CS → 10 kΩ → 3V3 nếu module chưa có pull-up. Không nối cứng CS lên 3V3 khi dùng SPI.",
        "SCX → 10 kΩ → GND; SDX → 10 kΩ → GND, chỉ cho module không nối cảm biến phụ.",
        "INT2 và các chân NC: để trống. Không nối các chân INT xuống GND.",
        "SA0 đang là MISO: không nối cứng GND/3V3; bỏ jumper kéo cứng nếu module có.",
    ]
    for i, s in enumerate(notes):
        txt(35, 311 - i * 29, s, 10)

    txt(35, 178, "Cấu hình đề xuất để kiểm chứng", 13, weight="bold")
    txt(35, 148, "SPI mode 3 · 1 MHz · WHO 0x70: HAODR 200 Hz · WHO 0x71: 240 Hz · ±4 g / ±500°/s", 10)
    txt(35, 119, "Dây 5–10 cm; mass sát dây tín hiệu; nguồn/logic 3,3 V. Motor và UART MKS giữ nguyên.", 10)
    txt(35, 76, "NẠP PROFILE LSM6DSV TRƯỚC KHI SỬ DỤNG", 12, red, weight="bold")
    txt(35, 48, "Profile lolin_s3_mini_lsm6dsv dùng GPIO8 CS và GPIO9 INPUT/MISO.", 10, red)
    txt(35, 16, "Nguồn: ST DS13476 Rev 5 + PinConfig.h + ảnh pinmap. Chưa kiểm chứng trên module thật.", 9, muted)

    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    for suffix in ("png", "svg"):
        fig.savefig(out / f"ket_noi_lsm6dsv_spi.{suffix}", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    main()
