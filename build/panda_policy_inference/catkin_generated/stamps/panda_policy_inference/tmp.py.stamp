#!/usr/bin/env python3

import argparse
import base64
import mimetypes
from pathlib import Path
from typing import Literal

from openai import OpenAI
from pydantic import BaseModel, Field
import time

MODEL_NAME = "gpt-5.6-terra"


class ActionDecision(BaseModel):
    action: Literal[
        "MOVE_IMAGE_LEFT",
        "MOVE_IMAGE_RIGHT",
        "MOVE_IMAGE_UP",
        "MOVE_IMAGE_DOWN",
        "STOP",
        "UNCERTAIN",
    ]

    confidence: float = Field(ge=0.0, le=1.0)
    reason: str


def image_to_data_url(image_path: Path) -> str:
    mime_type, _ = mimetypes.guess_type(image_path.name)

    if mime_type not in {"image/jpeg", "image/png"}:
        raise ValueError("The input must be a JPEG or PNG image.")

    encoded_image = base64.b64encode(
        image_path.read_bytes()
    ).decode("utf-8")

    return f"data:{mime_type};base64,{encoded_image}"


def select_action(
    image_path: Path,
    task: str,
) -> ActionDecision:
    client = OpenAI(
        timeout=30.0,
        max_retries=1,
    )

    image_data_url = image_to_data_url(image_path)

    response = client.responses.parse(
        model=MODEL_NAME,
        reasoning={
            "effort": "none", #"low",
        },
        input=[
            {
                "role": "system",
                "content": (
                    "You are a visual action selector for a robot experiment. "
                    "You only propose one discrete action. "
                    "Do not assume depth from a single RGB image. "
                    "Return UNCERTAIN if the target or gripper is unclear."
                ),
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": (
                            f"Task: {task}\n\n"
                            "Inspect the image and determine how the gripper "
                            "should move in the image plane to approach the "
                            "target. Choose exactly one allowed action.\n\n"
                            "MOVE_IMAGE_LEFT: move the gripper toward image-left.\n"
                            "MOVE_IMAGE_RIGHT: move toward image-right.\n"
                            "MOVE_IMAGE_UP: move toward the top of the image.\n"
                            "MOVE_IMAGE_DOWN: move toward the bottom of the image.\n"
                            "STOP: the gripper is sufficiently aligned.\n"
                            "UNCERTAIN: the target or gripper cannot be identified."
                        ),
                    },
                    {
                        "type": "input_image",
                        "image_url": image_data_url,
                        "detail": "auto",
                    },
                ],
            },
        ],
        text_format=ActionDecision,
    )

    print("API usage:")
    print(response.usage)

    if response.output_parsed is None:
        return ActionDecision(
            action="UNCERTAIN",
            confidence=0.0,
            reason="The API did not return a parsed decision.",
        )

    return response.output_parsed


def main():
    parser = argparse.ArgumentParser(
        description="Select a discrete robot action using GPT-5.6 Terra."
    )

    parser.add_argument(
        "image",
        type=Path,
        help="Path to a JPEG or PNG camera image.",
    )

    parser.add_argument(
        "--task",
        default="Move the gripper above the red block.",
        help="Natural-language robot task.",
    )

    args = parser.parse_args()

    if not args.image.is_file():
        raise FileNotFoundError(
            f"Image does not exist: {args.image}"
        )

    decision = select_action(
        image_path=args.image,
        task=args.task,
    )

    print("\nDecision:")
    print(decision.model_dump_json(indent=2))


if __name__ == "__main__":
    main()