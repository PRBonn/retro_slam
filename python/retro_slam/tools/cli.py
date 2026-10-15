# MIT License
#
# Copyright (c) 2026 Saurabh Gupta.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import glob
import os
from pathlib import Path
from typing import Optional

import typer

from retro_slam.datasets import available_dataloaders, sequence_dataloaders


def guess_dataloader(data: Path, default_dataloader: str):
    """Guess which dataloader to use in case the user didn't specify with --dataloader flag.

    TODO: Avoid having to return again the data Path. But when guessing multiple .bag files or the
    metadata.yaml file, we need to change the Path specifed by the user.
    """
    if data.is_file():
        if data.name == "metadata.yaml":
            return "rosbag", data.parent  # database is in directory, not in .yml
        if data.name.split(".")[-1] in "bag":
            return "rosbag", data
        if data.name.split(".")[-1] == "pcap":
            return "ouster", data
        if data.name.split(".")[-1] == "mcap":
            return "mcap", data
    elif data.is_dir():
        if (data / "metadata.yaml").exists():
            # a directory with a metadata.yaml must be a ROS2 bagfile
            return "rosbag", data
        if len(glob.glob(os.path.join(data, "*.bag"))) > 0:
            return "rosbag", data
    return default_dataloader, data


def name_callback(value: str):
    if not value:
        return value
    dl = available_dataloaders()
    if value not in dl:
        raise typer.BadParameter(f"Supported dataloaders are:\n{', '.join(dl)}")
    return value


def version_callback(value: bool):
    if value:
        try:
            # Check that the python bindings are properly built and can be loaded at runtime
            from retro_slam.pybind import retro_slam_pybind
        except ImportError as e:
            print(80 * "*")
            print(f"[ERRROR] Python bindings not properly built! Please open a issue on github")
            print(f"[ERRROR] '{e}'")
            print(80 * "*")
            raise typer.Exit(1)

        import retro_slam

        print(f"Retro-SLAM Version: {retro_slam.__version__}")
        raise typer.Exit(0)


app = typer.Typer(add_completion=False, rich_markup_mode="rich")

# Remove from the help those dataloaders we explicitly say how to use
_available_dl_help = available_dataloaders()
_available_dl_help.remove("generic")
_available_dl_help.remove("mcap")
_available_dl_help.remove("ouster")
_available_dl_help.remove("rosbag")


@app.command()
def retro_slam(
    data: Path = typer.Argument(
        ...,
        help="The data directory used by the specified dataloader",
        show_default=False,
    ),
    dataloader: str = typer.Option(
        None,
        show_default=False,
        case_sensitive=False,
        autocompletion=available_dataloaders,
        callback=name_callback,
        help="[Optional] Use a specific dataloader from those supported by Retro-SLAM",
    ),
    visualize: bool = typer.Option(
        False,
        "--visualize",
        "-v",
        help="[Optional] Visualize Ground Truth Loop Closures in the data sequence",
        rich_help_panel="Additional Options",
    ),
    sequence: Optional[str] = typer.Option(
        None,
        "--sequence",
        "-s",
        show_default=False,
        help="[Optional] For some dataloaders, you need to specify a given sequence",
        rich_help_panel="Additional Options",
    ),
    topic: Optional[str] = typer.Option(
        None,
        "--topic",
        "-t",
        show_default=False,
        help="[Optional] Only valid when processing rosbag files",
        rich_help_panel="Additional Options",
    ),
    n_scans: int = typer.Option(
        -1,
        "--n-scans",
        "-n",
        show_default=False,
        help="[Optional] Specify the number of scans to process, default is the entire dataset",
        rich_help_panel="Additional Options",
    ),
    config: Optional[Path] = typer.Option(
        None,
        "--config",
        exists=True,
        show_default=False,
        help="[Optional] Path to the configuration file",
    ),
    meta: Optional[Path] = typer.Option(
        None,
        "--meta",
        "-m",
        exists=True,
        show_default=False,
        help="[Optional] For Ouster pcap dataloader, specify metadata json file path explicitly",
        rich_help_panel="Additional Options",
    ),
):
    # Attempt to guess some common file extensions to avoid using the --dataloader flag
    if not dataloader:
        dataloader, data = guess_dataloader(data, default_dataloader="generic")

    # Validate some options
    if dataloader in sequence_dataloaders() and sequence is None:
        print('You must specify a sequence "--sequence"')
        raise typer.Exit(code=1)

    from retro_slam.datasets import dataset_factory
    from retro_slam.pipeline import SlamPipeline

    SlamPipeline(
        dataset=dataset_factory(
            dataloader=dataloader,
            data_dir=data,
            sequence=sequence,
            topic=topic,
            meta=meta,
        ),
        config_file=config,
        visualize=visualize,
        n_scans=n_scans,
    ).run().print()


def run():
    app()


if __name__ == "__main__":
    run()
