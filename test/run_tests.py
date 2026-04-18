"""Run cocotb tests using cocotb.runner (Windows-compatible, no Make needed)."""
import os
from cocotb_tools.runner import get_runner

def main():
    src_dir = os.path.join(os.path.dirname(__file__), "..", "src")
    test_dir = os.path.dirname(__file__)

    runner = get_runner("icarus")
    runner.build(
        sources=[
            os.path.join(src_dir, "project.v"),
            os.path.join(src_dir, "spi_slave.v"),
            os.path.join(test_dir, "tb.v"),
        ],
        hdl_toplevel="tb",
        build_args=["-g2012", f"-I{src_dir}"],
    )
    runner.test(
        hdl_toplevel="tb",
        test_module="test",
    )

if __name__ == "__main__":
    main()
