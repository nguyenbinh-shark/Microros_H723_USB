"""Map file operations shared by host tests and the SSH helper (standard library + PyYAML)."""
import base64
import hashlib
import math
import os
from pathlib import Path
import re
import struct
import uuid
import zlib


def map_name(name):
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
        raise ValueError("Tên map dùng 1–64 chữ không dấu, số, dấu - hoặc _")
    return name


def contained(directory, path):
    directory, path = Path(directory).resolve(), Path(path).resolve()
    if not path.is_relative_to(directory):
        raise ValueError("File map nằm ngoài thư mục bản đồ")
    return path


def read_pgm(raw):
    index = 0
    def token():
        nonlocal index
        while index < len(raw):
            if raw[index:index+1] in b" \t\r\n":
                index += 1
            elif raw[index:index+1] == b"#":
                end = raw.find(b"\n", index)
                index = len(raw) if end < 0 else end + 1
            else:
                break
        start = index
        while index < len(raw) and raw[index:index+1] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise ValueError("PGM header không hợp lệ")
        return raw[start:index]
    kind, width, height, maximum = token(), int(token()), int(token()), int(token())
    if kind not in (b"P5", b"P2") or width <= 0 or height <= 0 or width * height > 20_000_000 or not 0 < maximum <= 65535:
        raise ValueError("PGM không được hỗ trợ hoặc quá lớn")
    count = width * height
    if kind == b"P5":
        if index >= len(raw) or raw[index:index+1] not in b" \t\r\n":
            raise ValueError("PGM thiếu dấu phân cách dữ liệu")
        index += 2 if raw[index:index+2] == b"\r\n" else 1
        size = count * (2 if maximum > 255 else 1)
        pixels = raw[index:index+size]
        if len(pixels) != size:
            raise ValueError("PGM thiếu dữ liệu pixel")
        if maximum > 255:
            values = struct.unpack(">" + "H" * count, pixels)
            pixels = bytes(round(value * 255 / maximum) for value in values)
        elif maximum != 255:
            pixels = bytes(round(value * 255 / maximum) for value in pixels)
    else:
        values = [int(token()) for _ in range(count)]
        if any(value < 0 or value > maximum for value in values):
            raise ValueError("PGM có pixel ngoài miền giá trị")
        pixels = bytes(round(value * 255 / maximum) for value in values)
    return width, height, pixels


def thumbnail(raw, max_width=600, max_height=380):
    width, height, pixels = read_pgm(raw)
    ratio = min(1.0, max_width / width, max_height / height)
    out_w, out_h = max(1, round(width * ratio)), max(1, round(height * ratio))
    rows = bytearray()
    for y in range(out_h):
        rows.append(0)  # PNG filter: None
        row = min(height - 1, int(y / ratio)) * width
        rows.extend(pixels[row + min(width - 1, int(x / ratio))] for x in range(out_w))
    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", out_w, out_h, 8, 0, 0, 0, 0))
    return png + chunk(b"IDAT", zlib.compress(bytes(rows))) + chunk(b"IEND", b"")


def map_bundle(directory, name):
    import yaml
    directory = Path(directory).resolve()
    candidate = directory / (map_name(name) + ".yaml")
    if candidate.is_symlink():
        raise ValueError("Không sửa map qua symlink")
    metadata_path = contained(directory, candidate)
    if metadata_path.stat().st_size > 1_000_000:
        raise ValueError("YAML quá lớn")
    metadata_raw = metadata_path.read_bytes()
    metadata = yaml.safe_load(metadata_raw)
    if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
        raise ValueError("Map YAML thiếu image")
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or not all(math.isfinite(float(x)) for x in origin):
        raise ValueError("Thông số map không hợp lệ")
    candidate = metadata_path.parent / metadata["image"]
    if candidate.is_symlink():
        raise ValueError("Không sửa ảnh map qua symlink")
    image_path = contained(directory, candidate)
    if image_path.suffix.lower() != ".pgm" or image_path.stat().st_size > 64_000_000:
        raise ValueError("Cần file PGM, tối đa 64 MB")
    image_raw = image_path.read_bytes()
    width, height, _ = read_pgm(image_raw)
    info = dict(name=name, width=width, height=height, resolution=resolution,
                origin=origin, modified=metadata_path.stat().st_mtime,
                bytes=len(metadata_raw) + len(image_raw),
                revision=hashlib.sha256(metadata_raw + b"\0" + image_raw).hexdigest())
    return info, metadata_path, image_path, metadata, image_raw


def list_maps(directory):
    result = []
    for path in sorted(Path(directory).glob("*.yaml")):
        try:
            result.append(map_bundle(directory, path.stem)[0])
        except Exception as exc:
            result.append(dict(name=path.stem, error=str(exc)))
    return result


def export_bundle(bundle, name):
    import yaml
    info, _, _, metadata, image = bundle
    metadata = dict(metadata, image=name + ".pgm")
    yaml_raw = yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True).encode("utf-8")
    return dict(name=name, image_name=name + ".pgm",
                yaml_b64=base64.b64encode(yaml_raw).decode("ascii"),
                image_b64=base64.b64encode(image).decode("ascii"),
                yaml_sha256=hashlib.sha256(yaml_raw).hexdigest(),
                image_sha256=hashlib.sha256(image).hexdigest(), revision=info["revision"])


def archive_bundle(directory, bundle):
    import yaml
    _, metadata_path, image_path, _, _ = bundle
    shared = False
    for path in Path(directory).glob("*.yaml"):
        if path.resolve() == metadata_path:
            continue
        try:
            other = yaml.safe_load(path.read_text())
            if (path.parent / other["image"]).resolve() == image_path:
                shared = True
        except Exception:
            # Unknown references should never cause an image to be removed.
            shared = True
    target = contained(directory, Path(directory) / ".trash" / uuid.uuid4().hex)
    target.mkdir(parents=True)
    moved_image = False
    try:
        if not shared:
            os.replace(image_path, target / image_path.name)
            moved_image = True
        os.replace(metadata_path, target / metadata_path.name)
    except Exception:
        if moved_image:
            os.replace(target / image_path.name, image_path)
        raise
    return str(target)


def map_command(directory, action, name=None, new_name=None, revision=None):
    directory = Path(directory).resolve()
    if action == "maps":
        return {"maps": list_maps(directory)}
    bundle = map_bundle(directory, name)
    if action == "map-preview":
        return dict(bundle[0], png_b64=base64.b64encode(thumbnail(bundle[4])).decode("ascii"))
    if action == "map-export":
        if revision and revision != bundle[0]["revision"]:
            raise ValueError("Map đã thay đổi; hãy tải lại danh sách trước khi tải về")
        return export_bundle(bundle, name)
    if action not in ("map-rename", "map-delete"):
        raise ValueError("Unknown map action")
    if not revision or bundle[0]["revision"] != revision:
        raise ValueError("Map đã thay đổi; hãy tải lại danh sách trước khi sửa")
    if action == "map-delete":
        return {"archived": name, "trash": archive_bundle(directory, bundle)}
    new_name = map_name(new_name)
    exported = export_bundle(bundle, new_name)
    outputs = [(directory / (new_name + ".pgm"), base64.b64decode(exported["image_b64"])),
               (directory / (new_name + ".yaml"), base64.b64decode(exported["yaml_b64"]))]
    if any(path.exists() for path, _ in outputs):
        raise FileExistsError("Tên map mới đã tồn tại")
    written = []
    try:
        for path, raw in outputs:
            with path.open("xb") as file:
                written.append(path)
                file.write(raw)
    except Exception:
        for path in written:
            path.unlink()
        raise
    archive_bundle(directory, bundle)
    return {"renamed": name, "name": new_name}
