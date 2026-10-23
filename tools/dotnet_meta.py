"""List public types with typed method / property signatures of a .NET assembly by parsing its
ECMA-335 metadata directly (reads bytes only; never loads or runs the assembly).

    python dotnet_meta.py ASSEMBLY.dll [...] [--ns PREFIX] [--match REGEX] [--type REGEX] [--all]

--match keeps types whose full name, member names or property names match.
--type keeps only types whose full name matches.
"""
import re
import struct
import sys

PRIM = {0x01: "void", 0x02: "bool", 0x03: "char", 0x04: "sbyte", 0x05: "byte", 0x06: "short", 0x07: "ushort",
        0x08: "int", 0x09: "uint", 0x0A: "long", 0x0B: "ulong", 0x0C: "float", 0x0D: "double", 0x0E: "string",
        0x16: "TypedReference", 0x18: "IntPtr", 0x19: "UIntPtr", 0x1C: "object"}


class Meta:
    def __init__(self, path):
        self.b = b = open(path, "rb").read()
        pe = struct.unpack_from("<I", b, 0x3C)[0]
        assert b[pe:pe + 4] == b"PE\0\0", "not a PE file"
        nsec = struct.unpack_from("<H", b, pe + 6)[0]
        opt_size = struct.unpack_from("<H", b, pe + 20)[0]
        opt = pe + 24
        magic = struct.unpack_from("<H", b, opt)[0]
        dd = opt + (96 if magic == 0x10B else 112)
        cli_rva = struct.unpack_from("<I", b, dd + 14 * 8)[0]
        if not cli_rva:
            raise ValueError("not a .NET assembly")
        self.sections = []
        sec = opt + opt_size
        for i in range(nsec):
            vsize, va, rawsize, rawptr = struct.unpack_from("<IIII", b, sec + i * 40 + 8)
            self.sections.append((va, max(vsize, rawsize), rawptr))
        md = self.off(struct.unpack_from("<I", b, self.off(cli_rva) + 8)[0])
        vlen = struct.unpack_from("<I", b, md + 12)[0]
        p = md + 16 + vlen + 2
        nstreams = struct.unpack_from("<H", b, p)[0]
        p += 2
        self.streams = {}
        for _ in range(nstreams):
            o, size = struct.unpack_from("<II", b, p)
            p += 8
            end = b.index(b"\0", p)
            self.streams[b[p:end].decode()] = (md + o, size)
            p = (end + 4) & ~3
        self.parse_tables()

    def off(self, rva):
        for va, size, raw in self.sections:
            if va <= rva < va + size:
                return rva - va + raw
        raise ValueError(f"rva {rva:#x} not in a section")

    def string(self, idx):
        start = self.streams["#Strings"][0] + idx
        return self.b[start:self.b.index(b"\0", start)].decode("utf-8", "replace")

    @staticmethod
    def cint(buf, pos):
        b0 = buf[pos]
        if b0 & 0x80 == 0:
            return b0, pos + 1
        if b0 & 0xC0 == 0x80:
            return ((b0 & 0x3F) << 8) | buf[pos + 1], pos + 2
        return ((b0 & 0x1F) << 24) | (buf[pos + 1] << 16) | (buf[pos + 2] << 8) | buf[pos + 3], pos + 4

    def blob(self, idx):
        p = self.streams["#Blob"][0] + idx
        n, h = self.cint(self.b, p)
        return self.b[h:h + n]

    def parse_tables(self):
        b = self.b
        p = self.streams.get("#~", self.streams.get("#-"))[0]
        heap_sizes = b[p + 6]
        valid = struct.unpack_from("<Q", b, p + 8)[0]
        p += 24
        rows = {}
        for t in range(64):
            if valid >> t & 1:
                rows[t] = struct.unpack_from("<I", b, p)[0]
                p += 4
        if heap_sizes & 0x40:
            p += 4
        self.rows = rows
        si = 4 if heap_sizes & 1 else 2
        gi = 4 if heap_sizes & 2 else 2
        bi = 4 if heap_sizes & 4 else 2

        def idx(table):
            return 4 if rows.get(table, 0) > 0xFFFF else 2

        def coded(tables, bits):
            m = max(rows.get(t, 0) for t in tables if t is not None)
            return 4 if m >= (1 << (16 - bits)) else 2

        TDOR = coded([2, 1, 27], 2)
        HASCONST = coded([4, 8, 23], 2)
        HASCA = coded([6, 4, 1, 2, 8, 9, 10, 0, 14, 23, 20, 17, 26, 27, 32, 35, 38, 39, 40, 42, 44, 43], 5)
        HASFM = coded([4, 8], 1)
        HASDS = coded([2, 6, 32], 2)
        MRP = coded([2, 1, 26, 6, 27], 3)
        HASSEM = coded([20, 23], 1)
        MDOR = coded([6, 10], 1)
        MFWD = coded([4, 6], 1)
        IMPL = coded([38, 35, 39], 2)
        CAT = coded([None, None, 6, 10, None], 3)
        RS = coded([0, 26, 35, 1], 2)
        TOMD = coded([2, 6], 1)
        schema = {
            0: [2, si, gi, gi, gi], 1: [RS, si, si], 2: [4, si, si, TDOR, idx(4), idx(6)], 3: [idx(4)],
            4: [2, si, bi], 5: [idx(6)], 6: [4, 2, 2, si, bi, idx(8)], 7: [idx(8)], 8: [2, 2, si],
            9: [idx(2), TDOR], 10: [MRP, si, bi], 11: [2, HASCONST, bi], 12: [HASCA, CAT, bi],
            13: [HASFM, bi], 14: [2, HASDS, bi], 15: [2, 4, idx(2)], 16: [4, idx(4)], 17: [bi],
            18: [idx(2), idx(20)], 19: [idx(20)], 20: [2, si, TDOR], 21: [idx(2), idx(23)],
            22: [idx(23)], 23: [2, si, bi], 24: [2, idx(6), HASSEM], 25: [idx(2), MDOR, MDOR],
            26: [si], 27: [bi], 28: [2, MFWD, si, idx(26)], 29: [4, idx(4)], 30: [4, 4],
            31: [4], 32: [4, 2, 2, 2, 2, 4, bi, si, si], 33: [4], 34: [4, 4, 4],
            35: [2, 2, 2, 2, 4, bi, si, si, bi], 36: [4, idx(35)], 37: [4, 4, 4, idx(35)],
            38: [4, si, bi], 39: [4, 4, si, si, IMPL], 40: [4, 4, IMPL], 41: [idx(2), idx(2)],
            42: [2, 2, TOMD, si], 43: [MDOR, bi], 44: [idx(42), TDOR],
        }
        self.tables = {}
        for t in sorted(rows):
            recs = []
            for _ in range(rows[t]):
                rec = []
                for size in schema[t]:
                    rec.append(struct.unpack_from("<I" if size == 4 else "<H", b, p)[0])
                    p += size
                recs.append(rec)
            self.tables[t] = recs

    # ---- signatures ----
    def tdor_name(self, coded):
        tag, i = coded & 3, coded >> 2
        if i == 0:
            return "?"
        if tag == 0:
            r = self.tables[2][i - 1]
            return self.string(r[1])
        if tag == 1:
            r = self.tables[1][i - 1]
            return self.string(r[1])
        sig = self.blob(self.tables[27][i - 1][0])
        return self.type_sig(sig, 0)[0]

    def type_sig(self, s, p):
        et = s[p]
        p += 1
        if et in PRIM:
            return PRIM[et], p
        if et in (0x11, 0x12):
            c, p = self.cint(s, p)
            return self.tdor_name(c).split("`")[0], p
        if et == 0x1D:
            t, p = self.type_sig(s, p)
            return t + "[]", p
        if et in (0x0F, 0x10):
            t, p = self.type_sig(s, p)
            return t + ("*" if et == 0x0F else "&"), p
        if et in (0x13, 0x1E):
            n, p = self.cint(s, p)
            return ("T" if et == 0x13 else "M") + str(n), p
        if et == 0x15:
            p += 1  # class / valuetype
            c, p = self.cint(s, p)
            n, p = self.cint(s, p)
            args = []
            for _ in range(n):
                a, p = self.type_sig(s, p)
                args.append(a)
            return f"{self.tdor_name(c).split('`')[0]}<{', '.join(args)}>", p
        if et == 0x14:
            t, p = self.type_sig(s, p)
            rank, p = self.cint(s, p)
            nsz, p = self.cint(s, p)
            for _ in range(nsz):
                _, p = self.cint(s, p)
            nlo, p = self.cint(s, p)
            for _ in range(nlo):
                _, p = self.cint(s, p)
            return t + "[" + "," * (rank - 1) + "]", p
        if et in (0x1F, 0x20):
            _, p = self.cint(s, p)
            return self.type_sig(s, p)
        if et == 0x45:
            return self.type_sig(s, p)
        if et == 0x1B:
            return "fnptr", len(s)
        return f"et{et:#x}", len(s)

    def method_sig(self, blob):
        try:
            p = 0
            flags = blob[p]; p += 1
            if flags & 0x10:
                _, p = self.cint(blob, p)
            n, p = self.cint(blob, p)
            ret, p = self.type_sig(blob, p)
            params = []
            for _ in range(n):
                t, p = self.type_sig(blob, p)
                params.append(t)
            return ret, params
        except Exception:
            return "?", []

    def prop_type(self, blob):
        try:
            p = 1
            _, p = self.cint(blob, p)
            return self.type_sig(blob, p)[0]
        except Exception:
            return "?"

    def types(self):
        td, md = self.tables.get(2, []), self.tables.get(6, [])
        fields, params = self.tables.get(4, []), self.tables.get(8, [])
        props, pmap = self.tables.get(23, []), self.tables.get(21, [])
        prop_owner = {}
        for k, (parent, plist) in enumerate(pmap):
            end = pmap[k + 1][1] if k + 1 < len(pmap) else len(props) + 1
            prop_owner[parent] = range(plist, end)
        out = []
        for i, (flags, name, ns, extends, field_list, method_list) in enumerate(td):
            m_end = td[i + 1][5] if i + 1 < len(td) else len(md) + 1
            f_end = td[i + 1][4] if i + 1 < len(td) else len(fields) + 1
            methods = []
            for mi in range(method_list, m_end):
                _rva, _impl, mflags, mname, sig, param_list = md[mi - 1]
                p_end = md[mi][5] if mi < len(md) else len(params) + 1
                pnames = {params[pi - 1][1]: self.string(params[pi - 1][2]) for pi in range(param_list, p_end)}
                ret, ptypes = self.method_sig(self.blob(sig))
                args = [f"{t} {pnames.get(k + 1, '_')}" for k, t in enumerate(ptypes)]
                methods.append((mflags, self.string(mname), ret, args))
            flds = [(fields[fi - 1][0], self.string(fields[fi - 1][1])) for fi in range(field_list, f_end)]
            prop_list = [(self.string(props[j - 1][1]), self.prop_type(self.blob(props[j - 1][2]))) for j in prop_owner.get(i + 1, [])]
            base = self.tdor_name(extends).split("`")[0] if extends else ""
            out.append({"flags": flags, "ns": self.string(ns), "name": self.string(name), "base": base,
                        "methods": methods, "fields": flds, "props": prop_list})
        return out


def main():
    args = sys.argv[1:]
    opts = {}
    for flag in ("--ns", "--match", "--type"):
        if flag in args:
            i = args.index(flag)
            opts[flag] = args[i + 1]
            del args[i:i + 2]
    show_all = "--all" in args
    args = [a for a in args if a != "--all"]
    match = re.compile(opts["--match"], re.I) if "--match" in opts else None
    tmatch = re.compile(opts["--type"], re.I) if "--type" in opts else None
    for path in args:
        m = Meta(path)
        print(f"##### {path}")
        for t in m.types():
            public = (t["flags"] & 7) in (1, 2)
            if not (public or show_all) or t["name"].startswith("<"):
                continue
            full = f"{t['ns']}.{t['name']}" if t["ns"] else t["name"]
            if "--ns" in opts and not full.startswith(opts["--ns"]):
                continue
            if tmatch and not tmatch.search(full):
                continue
            meths = [x for x in t["methods"] if (x[0] & 7) == 6 or show_all]
            if match and not (match.search(full) or any(match.search(x[1]) for x in meths) or any(match.search(n) for n, _ in t["props"])):
                continue
            print(f"\n{full}" + (f" : {t['base']}" if t["base"] and t["base"] not in ("Object",) else ""))
            for n, ty in t["props"]:
                if "." not in n:
                    print(f"  prop {ty} {n}")
            pub_fields = [n for f, n in t["fields"] if (f & 7) == 6]
            if pub_fields:
                print("  fields: " + ", ".join(pub_fields[:40]))
            seen = set()
            for _, n, ret, a in meths:
                if n.startswith(("get_", "set_", "add_", "remove_")) or (n, tuple(a)) in seen:
                    continue
                seen.add((n, tuple(a)))
                print(f"  {ret} {n}({', '.join(a)})")


if __name__ == "__main__":
    main()
