import base64
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import map_library as maps
from rpbot_console import write_download


class MapLibraryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "maps"
        self.root.mkdir()
        (self.root / "room.pgm").write_bytes(b"P5\n# map\n2 2\n255\n" + bytes([0, 205, 254, 255]))
        (self.root / "room.yaml").write_text("image: room.pgm\nresolution: 0.05\norigin: [-1, -2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    def revision(self):
        return maps.map_command(self.root, "maps")["maps"][0]["revision"]

    def test_preview_is_valid_png_with_original_orientation(self):
        response = maps.map_command(self.root, "map-preview", "room")
        raw = base64.b64decode(response["png_b64"])
        self.assertTrue(raw.startswith(b"\x89PNG\r\n\x1a\n"))
        offset, pixels = 8, b""
        while offset < len(raw):
            size = struct.unpack(">I", raw[offset:offset+4])[0]
            kind, data = raw[offset+4:offset+8], raw[offset+8:offset+8+size]
            crc = struct.unpack(">I", raw[offset+8+size:offset+12+size])[0]
            self.assertEqual(zlib.crc32(kind + data) & 0xffffffff, crc)
            if kind == b"IDAT":
                pixels += data
            offset += size + 12
        self.assertEqual(zlib.decompress(pixels), bytes([0, 0, 205, 0, 254, 255]))

    def test_binary_pgm_does_not_strip_whitespace_pixels(self):
        raw = b"P5\r\n2 1\r\n255\r\n" + bytes([10, 32])
        self.assertEqual(maps.read_pgm(raw), (2, 1, bytes([10, 32])))

    def test_ascii_pgm_and_sixteen_bit_are_supported(self):
        self.assertEqual(maps.read_pgm(b"P2\n2 1\n15\n0 # note\n15\n")[2], bytes([0, 255]))
        self.assertEqual(maps.read_pgm(b"P5\n2 1\n65535\n\x00\x00\xff\xff")[2], bytes([0, 255]))

    def test_rename_updates_yaml_and_preserves_old_files_in_trash(self):
        import yaml
        original = (self.root / "room.pgm").read_bytes()
        maps.map_command(self.root, "map-rename", "room", "hall", self.revision())
        self.assertFalse((self.root / "room.yaml").exists())
        self.assertEqual((self.root / "hall.pgm").read_bytes(), original)
        metadata = yaml.safe_load((self.root / "hall.yaml").read_text())
        self.assertEqual(metadata["image"], "hall.pgm")
        self.assertEqual(metadata["origin"], [-1, -2, 0])
        self.assertEqual(len(list((self.root / ".trash").glob("*/room.yaml"))), 1)

    def test_rename_never_overwrites_an_existing_target(self):
        target = self.root / "hall.pgm"
        target.write_bytes(b"keep me")
        with self.assertRaises(FileExistsError):
            maps.map_command(self.root, "map-rename", "room", "hall", self.revision())
        self.assertEqual(target.read_bytes(), b"keep me")
        self.assertTrue((self.root / "room.yaml").exists())

    def test_changed_map_cannot_be_deleted_or_renamed_with_stale_revision(self):
        revision = self.revision()
        with (self.root / "room.yaml").open("a") as file:
            file.write("# updated\n")
        for action in ("map-delete", "map-rename", "map-export"):
            with self.assertRaisesRegex(ValueError, "Map đã thay đổi"):
                maps.map_command(self.root, action, "room", "hall", revision)
        self.assertTrue((self.root / "room.yaml").exists())

    def test_delete_shared_map_retains_image_for_other_map(self):
        (self.root / "second.yaml").write_bytes((self.root / "room.yaml").read_bytes())
        maps.map_command(self.root, "map-delete", "room", revision=self.revision())
        self.assertTrue((self.root / "room.pgm").exists())
        self.assertEqual(maps.map_command(self.root, "map-preview", "second")["width"], 2)

    def test_image_path_cannot_escape_map_directory(self):
        (self.root / "room.yaml").write_text("image: ../outside.pgm\nresolution: 0.1\norigin: [0,0,0]\n")
        with self.assertRaisesRegex(ValueError, "nằm ngoài"):
            maps.map_command(self.root, "map-export", "room")
        with self.assertRaises(ValueError):
            maps.map_command(self.root, "map-preview", "../room")

    def test_broken_yaml_is_reported_without_hiding_good_maps(self):
        (self.root / "broken.yaml").write_text("image: [\n")
        entries = maps.list_maps(self.root)
        self.assertEqual(len(entries), 2)
        self.assertIn("error", entries[0])
        self.assertEqual(entries[1]["name"], "room")

    def test_download_verifies_content_and_has_relative_image_path(self):
        import yaml
        payload = maps.map_command(self.root, "map-export", "room")
        destination = Path(self.temp.name) / "downloads"
        destination.mkdir()
        result = write_download(payload, destination)
        self.assertEqual((result / "room.pgm").read_bytes(), (self.root / "room.pgm").read_bytes())
        self.assertEqual(yaml.safe_load((result / "room.yaml").read_text())["image"], "room.pgm")
        with self.assertRaises(FileExistsError):
            write_download(payload, destination)
        payload["image_b64"] = base64.b64encode(b"corrupt").decode()
        with self.assertRaisesRegex(ValueError, "checksum"):
            write_download(payload, destination)


if __name__ == "__main__":
    unittest.main()
