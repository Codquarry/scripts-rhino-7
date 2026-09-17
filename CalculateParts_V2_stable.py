
# -*- coding: utf-8 -*-
import rhinoscriptsyntax as rs
import Rhino
import codecs
import math

def get_arc_bend_angle(arc):
    """Угол разворота дуги в радианах."""
    try:
        angle = arc.Angle
        if angle and angle > 1e-9:
            return angle
    except:
        pass
    if arc.Radius > 1e-9:
        return arc.Length / arc.Radius
    return 0.0


def get_radial_extents(brep, plane):
    """
    Минимальное и максимальное расстояние точек детали от оси гиба
    (ось = нормаль плоскости дуги, проходящая через центр дуги).

    Минимум - это внутренний радиус изгиба, максимум - внешний.
    Считается по самой геометрии, поэтому работает даже тогда, когда
    в модели распознана только одна кромка-дуга (например, внешняя).
    """
    points = []
    try:
        for v in brep.Vertices:
            points.append(v.Location)
    except:
        pass

    if len(points) < 2:
        points = []
        try:
            for edge in brep.Edges:
                points.append(edge.PointAtStart)
                points.append(edge.PointAtEnd)
                dom = edge.Domain
                points.append(edge.PointAt((dom.Min + dom.Max) / 2.0))
        except:
            pass

    r_min = None
    r_max = None
    for pt in points:
        vec = pt - plane.Origin
        h = vec * plane.ZAxis
        radial = vec - plane.ZAxis * h
        r = radial.Length
        if r_min is None or r < r_min:
            r_min = r
        if r_max is None or r > r_max:
            r_max = r

    return r_min, r_max


def get_oriented_dimensions(obj_id):
    brep = rs.coercebrep(obj_id)
    if not brep: return None, False
    
    arcs = []
    lines = []
    for edge in brep.Edges:
        is_arc = False
        try:
            res, arc = edge.TryGetArc()
            if res:
                arcs.append(arc)
                is_arc = True
        except:
            pass
            
        if is_arc:
            continue
            
        if edge.IsLinear():
            pt0 = edge.PointAtStart
            pt1 = edge.PointAtEnd
            vec = pt1 - pt0
            length = vec.Length
            if length > 0.01:
                lines.append((vec, length, pt0, pt1))
            continue
            
        # Если это сплайн, попробуем аппроксимировать его дугой по 3 точкам
        pt0 = edge.PointAtStart
        pt1 = edge.PointAtEnd
        try:
            success, mid_t = edge.NormalizedLengthParameter(0.5)
            pt_mid = edge.PointAt(mid_t) if success else edge.PointAt((edge.Domain.Min + edge.Domain.Max) / 2.0)
        except:
            mid_t = (edge.Domain.Min + edge.Domain.Max) / 2.0
            pt_mid = edge.PointAt(mid_t)
        
        vec = pt1 - pt0
        length = vec.Length
        if length > 0.01:
            # Проверяем, не является ли линия прямой (отклонение середины)
            dist_mid = pt_mid.DistanceTo(pt0 + vec * 0.5)
            if dist_mid < 0.1:
                lines.append((vec, length, pt0, pt1))
                continue
                
            # Попробуем создать дугу по 3 точкам
            try:
                arc = Rhino.Geometry.Arc(pt0, pt_mid, pt1)
                # Проверим отклонение в нескольких точках
                is_valid_arc = True
                for t in [0.25, 0.75]:
                    test_t = edge.Domain.Min + (edge.Domain.Max - edge.Domain.Min) * t
                    test_pt = edge.PointAt(test_t)
                    if arc.ClosestPoint(test_pt).DistanceTo(test_pt) > length * 0.1: # 10% tolerance for freeform curves
                        is_valid_arc = False
                        break
                if is_valid_arc:
                    arcs.append(arc)
            except:
                pass
                
    max_arc_len = max([a.Length for a in arcs]) if arcs else 0
    max_line_len = max([l[1] for l in lines]) if lines else 0

    if arcs and max_arc_len > max_line_len:
        long_arcs = [a for a in arcs if a.Length > max_arc_len * 0.5]
        if not long_arcs:
            long_arcs = arcs

        long_arcs.sort(key=lambda a: a.Radius)
        inner_arc = long_arcs[0]
        outer_arc = long_arcs[-1]

        # Опорная дуга - самая длинная: у неё больше всего геометрии,
        # поэтому её плоскость (центр гиба и ось гиба) определяется точнее всего.
        ref_arc = max(long_arcs, key=lambda a: a.Length)
        arc_plane = ref_arc.Plane
        bend_angle = get_arc_bend_angle(ref_arc)

        # Внешний радиус берём не по списку распознанных кромок (там может
        # оказаться только внутренняя дуга), а по реальным габаритам детали
        # относительно оси гиба: самая дальняя от оси точка и есть внешний радиус.
        r_min, r_max = get_radial_extents(brep, arc_plane)

        if r_min is not None and r_max is not None and (r_max - r_min) > 0.01:
            radius = r_max
            dim1 = r_max - r_min
        else:
            radius = outer_arc.Radius
            dim1 = outer_arc.Radius - inner_arc.Radius

        # Длина заготовки считается по внешней длинной стороне: L = R_внеш * угол.
        arc_length = radius * bend_angle
        if arc_length <= 0.01:
            arc_length = outer_arc.Length

        bbox = brep.GetBoundingBox(arc_plane)
        dim2 = bbox.Max.Z - bbox.Min.Z
        
        # Если размер перпендикулярно дуге (dim2) значительно больше длины самой дуги,
        # значит эта дуга - скругление в сечении, а не продольный изгиб профиля.
        if dim2 <= max_arc_len * 1.5:
            thickness = int(round(min(dim1, dim2)))
            width = int(round(max(dim1, dim2)))
            
            if thickness == 0 or width == 0:
                line_lengths = [l[1] for l in lines]
                line_lengths = sorted(list(set([int(round(l)) for l in line_lengths])))
                if len(line_lengths) >= 2:
                    thickness = line_lengths[0]
                    width = line_lengths[1]
                elif len(line_lengths) == 1:
                    thickness = line_lengths[0]
                    width = line_lengths[0]
                    
            return [thickness, width, arc_length, radius], True

    vectors = []
    for line in lines:
        vec = line[0]
        length = line[1]
        vec.Unitize()
        vectors.append((vec, length))
                
    if not vectors:
        bbox = brep.GetBoundingBox(Rhino.Geometry.Plane.WorldXY)
        if not bbox: return None, False
        dx = bbox.Max.X - bbox.Min.X
        dy = bbox.Max.Y - bbox.Min.Y
        dz = bbox.Max.Z - bbox.Min.Z
        return sorted([dx, dy, dz]), False
        
    vectors.sort(key=lambda x: x[1], reverse=True)
    best_vec = vectors[0][0]
    
    ortho_vec = None
    for vec, length in vectors:
        if abs(best_vec * vec) < 1e-4:
            ortho_vec = vec
            break
            
    if not ortho_vec:
        plane = Rhino.Geometry.Plane(Rhino.Geometry.Point3d.Origin, best_vec)
        ortho_vec = plane.XAxis
        
    origin = Rhino.Geometry.Point3d.Origin
    plane = Rhino.Geometry.Plane(origin, best_vec, ortho_vec)
    
    bbox = brep.GetBoundingBox(plane)
    dx = bbox.Max.X - bbox.Min.X
    dy = bbox.Max.Y - bbox.Min.Y
    dz = bbox.Max.Z - bbox.Min.Z
    
    return sorted([dx, dy, dz]), False

def calculate_parts():
    """
    Скрипт для подсчета количества деталей и их габаритов (Сечение + Длина).
    Детали могут быть произвольно ориентированы в пространстве. Габариты 
    рассчитываются по локальным осям объектов. Поддерживаются радиусные профили.
    """
    
    objects = rs.GetObjects("Выберите детали для расчета (Polysurfaces/Surfaces)", rs.filter.polysurface | rs.filter.surface)
    
    if not objects:
        print("Объекты не выбраны.")
        return

    parts_data = {}
    curved_parts_data = {}
    sheets_data = {}
    curved_sheets_data = {}

    PROFILE_WEIGHTS = {
        (20, 20): 0.69,
        (25, 25): 0.82,
        (25, 40): 1.39,
        (40, 25): 1.39,
        (40, 40): 1.78,
        (30, 30): 1.7,
        (50, 50): 2.25,
        (50, 100): 3.46,
        (100, 50): 3.46
    }

    # Вес листовых материалов, кг/м2. Считан из плотности по открытым
    # справочным данным: фанера берёзовая и ДСП 650 кг/м3, вспененный ПВХ
    # 550 кг/м3, сталь 7850 кг/м3.
    SHEET_MATERIALS = {
        (u"Фанера", 15): 9.75,
        (u"Фанера", 12): 7.8,
        (u"Фанера", 9): 5.85,
        (u"Фанера", 6): 3.9,
        (u"ДСП", 16): 10.4,
        (u"ПВХ", 8): 4.4,
        (u"ПВХ", 5): 2.75,
        (u"ПВХ", 3): 1.65,
        (u"Сталь", 1): 7.85,
        (u"Сталь", 2): 15.7,
        (u"Сталь", 3): 23.55,
        (u"Сталь", 5): 39.25,
        (u"Сталь", 8): 62.8
    }

    # Толщины 3, 5 и 8 мм есть и у ПВХ, и у стали - по одной толщине их не
    # различить, поэтому для них смотрим имя слоя детали.
    MATERIAL_KEYWORDS = {
        u"Фанера": (u"фанер", u"plywood", u"fanera"),
        u"ДСП": (u"дсп", u"ldsp", u"chipboard"),
        u"ПВХ": (u"пвх", u"pvc"),
        u"Сталь": (u"сталь", u"стал", u"steel")
    }

    SHEET_THICKNESSES = set([t for (m, t) in SHEET_MATERIALS])

    def is_sheet(dims):
        """
        Деталь считается листовой, если её наименьший габарит совпадает с
        толщиной из базы, а средний габарит заметно больше толщины. Второе
        условие отсекает бруски и короткие обрезки профиля: у настоящего
        листа пласть минимум в 5 раз шире толщины и не меньше 50 мм.

        Толщина сверяется точно (округление до миллиметра), а не с допуском
        +-5 мм как у профилей: у листов 3, 5, 6, 8 и 9 мм такой допуск
        смешал бы все материалы между собой.
        """
        if int(round(dims[0])) not in SHEET_THICKNESSES:
            return False
        return dims[1] >= dims[0] * 5 and dims[1] >= 50

    def detect_sheet_material(thickness, layer_name):
        """
        Материал листа по толщине. Если толщина принадлежит только одному
        материалу, имя слоя не нужно. Для спорных 3, 5 и 8 мм ищем в имени
        слоя ключевое слово; если его нет, материал не угадываем, а
        возвращаем None - такая деталь попадёт в отчёт с пометкой.
        """
        candidates = sorted([m for (m, t) in SHEET_MATERIALS if t == thickness])
        if len(candidates) == 1:
            return candidates[0]
        name = (layer_name or u"").lower()
        for material in candidates:
            for keyword in MATERIAL_KEYWORDS.get(material, ()):
                if keyword in name:
                    return material
        return None

    for obj in objects:
        dims, is_curved = get_oriented_dimensions(obj)
        if not dims:
            bbox = rs.BoundingBox(obj)
            if not bbox:
                continue
            dx = rs.Distance(bbox[0], bbox[1])
            dy = rs.Distance(bbox[1], bbox[2])
            dz = rs.Distance(bbox[0], bbox[4])
            dims = sorted([dx, dy, dz])
            is_curved = False

        if is_sheet(dims):
            thickness = int(round(dims[0]))
            try:
                layer_name = rs.ObjectLayer(obj)
            except:
                layer_name = u""
            material = detect_sheet_material(thickness, layer_name)
            if is_curved:
                # dims гнутой детали: толщина, ширина, длина дуги, радиус.
                # Площадь считаем по развёртке: ширина * длина дуги.
                key = (material, thickness, int(round(dims[1])),
                       int(round(dims[2])), int(round(dims[3])))
                if key in curved_sheets_data:
                    curved_sheets_data[key] += 1
                else:
                    curved_sheets_data[key] = 1
            else:
                key = (material, thickness, int(round(dims[1])), int(round(dims[2])))
                if key in sheets_data:
                    sheets_data[key] += 1
                else:
                    sheets_data[key] = 1
            continue

        if is_curved:
            thickness = int(round(dims[0]))
            width = int(round(dims[1]))
            length = int(round(dims[2])) # длина дуги по внешней стороне
            radius = int(round(dims[3]))
            key = (thickness, width, length, radius)
            if key in curved_parts_data:
                curved_parts_data[key] += 1
            else:
                curved_parts_data[key] = 1
        else:
            thickness = int(round(dims[0]))
            width = int(round(dims[1]))
            length = int(round(dims[2]))
            key = (thickness, width, length)
            if key in parts_data:
                parts_data[key] += 1
            else:
                parts_data[key] = 1

    def match_profile(t, w, known_profiles, tol=5):
        # Допуск +-5 мм: деталь 50x54 опознаётся как профиль 50x50.
        # Перебор по отсортированным ключам, иначе при равном отклонении
        # (например 35x35 между 30x30 и 40x40) результат зависел бы от
        # порядка ключей в словаре и менялся от запуска к запуску.
        best_match = None
        min_diff = float('inf')
        for kt, kw in sorted(known_profiles):
            # check direct
            dt = abs(kt - t)
            dw = abs(kw - w)
            if dt <= tol and dw <= tol:
                if dt + dw < min_diff:
                    min_diff = dt + dw
                    best_match = (kt, kw)
            # check swapped
            dt_swap = abs(kt - w)
            dw_swap = abs(kw - t)
            if dt_swap <= tol and dw_swap <= tol:
                if dt_swap + dw_swap < min_diff:
                    min_diff = dt_swap + dw_swap
                    best_match = (kw, kt)
        return best_match if best_match else (t, w)

    profiles = {}
    for key, count in parts_data.items():
        t, w, l = key
        prof_key = match_profile(t, w, PROFILE_WEIGHTS.keys())
        if prof_key not in profiles:
            profiles[prof_key] = {}
        if l not in profiles[prof_key]:
            profiles[prof_key][l] = 0
        profiles[prof_key][l] += count
        
    # convert back to list format for profiles
    for prof_key in profiles:
        profiles[prof_key] = [(l, c) for l, c in profiles[prof_key].items()]

    sorted_profiles = sorted(profiles.keys())

    curved_profiles = {}
    for key, count in curved_parts_data.items():
        t, w, arc_len, r = key
        prof_key = match_profile(t, w, PROFILE_WEIGHTS.keys())
        if prof_key not in curved_profiles:
            curved_profiles[prof_key] = {}
        # grouping by (arc_len, r)
        sub_key = (arc_len, r)
        if sub_key not in curved_profiles[prof_key]:
            curved_profiles[prof_key][sub_key] = 0
        curved_profiles[prof_key][sub_key] += count

    # convert back
    for prof_key in curved_profiles:
        curved_profiles[prof_key] = [(al, rad, c) for (al, rad), c in curved_profiles[prof_key].items()]
        
    sorted_curved_profiles = sorted(curved_profiles.keys())

    # Листовые детали группируем по паре (материал, толщина), внутри - по
    # габариту пласти.
    sheets = {}
    for key, count in sheets_data.items():
        material, thickness, w, l = key
        grp_key = (material, thickness)
        if grp_key not in sheets:
            sheets[grp_key] = {}
        size_key = (w, l)
        if size_key not in sheets[grp_key]:
            sheets[grp_key][size_key] = 0
        sheets[grp_key][size_key] += count

    for grp_key in sheets:
        sheets[grp_key] = [(w, l, c) for (w, l), c in sheets[grp_key].items()]

    # Материал может быть None (спорная толщина, слой молчит) - такие группы
    # уходят в конец списка.
    sorted_sheets = sorted(sheets.keys(),
                           key=lambda g: (g[0] is None, g[0] or u"", g[1]))

    # Гнутые листы - та же пара (материал, толщина), внутри - ширина,
    # длина дуги и радиус.
    curved_sheets = {}
    for key, count in curved_sheets_data.items():
        material, thickness, w, arc_len, r = key
        grp_key = (material, thickness)
        if grp_key not in curved_sheets:
            curved_sheets[grp_key] = {}
        size_key = (w, arc_len, r)
        if size_key not in curved_sheets[grp_key]:
            curved_sheets[grp_key][size_key] = 0
        curved_sheets[grp_key][size_key] += count

    for grp_key in curved_sheets:
        curved_sheets[grp_key] = [(w, al, rad, c)
                                  for (w, al, rad), c in curved_sheets[grp_key].items()]

    sorted_curved_sheets = sorted(curved_sheets.keys(),
                                  key=lambda g: (g[0] is None, g[0] or u"", g[1]))

    def sheet_short_label(grp_key):
        material, thickness = grp_key
        if material is None:
            return u"Лист {} мм".format(thickness)
        return u"{} {} мм".format(material, thickness)

    def sheet_label(grp_key, weight_per_m2):
        if grp_key[0] is None:
            return u"Лист {} мм (уточнить материал)".format(grp_key[1])
        label = sheet_short_label(grp_key)
        if weight_per_m2 > 0:
            label += u" ({} кг/м2)".format(str(weight_per_m2).replace('.', ','))
        return label

    def curved_sheet_label(grp_key, weight_per_m2):
        if grp_key[0] is None:
            return u"Лист {} мм (гнутый, уточнить материал)".format(grp_key[1])
        if weight_per_m2 > 0:
            return u"{} (гнутый, {} кг/м2)".format(
                sheet_short_label(grp_key), str(weight_per_m2).replace('.', ','))
        return u"{} (гнутый)".format(sheet_short_label(grp_key))

    def build_summary(profiles_total, sheet_totals):
        """
        Итоговый блок: вес профилей одной строкой, затем вес каждого типа
        листового материала отдельной строкой. Плоские и гнутые детали
        одного материала складываются в одну строку.

        Общий вес конструкции выводим только когда в модели есть и профили,
        и листы: иначе такая строка просто повторяла бы единственную строку
        над собой.
        """
        rows = []
        if profiles_total > 0:
            rows.append((u"Общий вес профилей:", round(profiles_total, 2), True))

        sheets_total = 0.0
        for grp_key in sorted(sheet_totals.keys(),
                              key=lambda g: (g[0] is None, g[0] or u"", g[1])):
            weight = round(sheet_totals[grp_key], 2)
            sheets_total += sheet_totals[grp_key]
            rows.append((u"{}:".format(sheet_short_label(grp_key)), weight, False))

        if profiles_total > 0 and sheets_total > 0:
            rows.append((u"Общий вес конструкции:",
                         round(profiles_total + sheets_total, 2), True))
        return rows

    try:
        import System
        excel_type = System.Type.GetTypeFromProgID("Excel.Application")
        if excel_type:
            excel = System.Activator.CreateInstance(excel_type)
            excel.Visible = True
            wb = excel.Workbooks.Add()
            sh = wb.ActiveSheet
            
            xlLeft = -4131
            xlCenter = -4108
            xlContinuous = 1
            xlThin = 2
            xlThick = 4
            
            row = 1
            profiles_total_weight = 0.0
            sheet_totals = {}
            
            for i, prof_key in enumerate(sorted_profiles):
                if row > 1:
                    row += 1
                
                header_row = row
                sh.Cells(row, 1).Value = u"Профиль"
                sh.Cells(row, 2).Value = u"Длина"
                sh.Cells(row, 3).Value = u"Количество"
                sh.Cells(row, 4).Value = u"Вес"
                row += 1
                
                weight_per_m = PROFILE_WEIGHTS.get(prof_key, 0.0)
                profile_name = u"{}x{}".format(prof_key[0], prof_key[1])
                if weight_per_m > 0:
                    profile_name += u" ({} кг/м.п.)".format(str(weight_per_m).replace('.', ','))
                lengths = sorted(profiles[prof_key], key=lambda x: -x[0])
                zone_total_weight = 0.0
                
                for j, item in enumerate(lengths):
                    length = item[0]
                    count = item[1]
                    total_weight = 0.0
                    
                    if weight_per_m > 0:
                        total_weight = round((length / 1000.0) * weight_per_m * count, 2)
                        zone_total_weight += total_weight
                        profiles_total_weight += total_weight
                        
                    if j == 0:
                        sh.Cells(row, 1).Value = profile_name
                        cell_colors = {
                            (20, 20): 9221330,  
                            (25, 25): 9498256,  
                            (25, 40): 15128749, 
                            (40, 25): 15128749, 
                            (40, 40): 13882323  
                        }
                        if prof_key in cell_colors:
                            sh.Cells(row, 1).Interior.Color = cell_colors[prof_key]
                    else:
                        sh.Cells(row, 1).Value = ""
                        
                    sh.Cells(row, 2).Value = length
                    sh.Cells(row, 3).Value = count
                    sh.Cells(row, 4).Value = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m > 0 else u"Нет в базе"
                    row += 1
                    
                total_row = None
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sh.Cells(row, 3).Value = u"Итого:"
                    sh.Cells(row, 3).Font.Bold = True
                    sh.Cells(row, 4).Value = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    sh.Cells(row, 4).Font.Bold = True
                    total_row = row
                    row += 1
                    
                header_range = sh.Range(sh.Cells(header_row, 1), sh.Cells(header_row, 4))
                header_range.Font.Bold = True
                for edge in (7, 8, 9, 10, 11):
                    try:
                        header_range.Borders(edge).LineStyle = xlContinuous
                        header_range.Borders(edge).Weight = xlThick
                    except:
                        pass
                        
                if total_row:
                    total_range = sh.Range(sh.Cells(total_row, 3), sh.Cells(total_row, 4))
                    for edge in (7, 8, 9, 10, 11):
                        try:
                            total_range.Borders(edge).LineStyle = xlContinuous
                            total_range.Borders(edge).Weight = xlThick
                        except:
                            pass

            for i, prof_key in enumerate(sorted_curved_profiles):
                if row > 1:
                    row += 1
                
                header_row = row
                sh.Cells(row, 1).Value = u"Профиль"
                sh.Cells(row, 2).Value = u"Длина"
                sh.Cells(row, 3).Value = u"Радиус (внешн.)"
                sh.Cells(row, 4).Value = u"Количество"
                sh.Cells(row, 5).Value = u"Вес"
                row += 1
                
                weight_per_m = PROFILE_WEIGHTS.get(prof_key, 0.0)
                profile_name = u"{}x{} (Радиусный)".format(prof_key[0], prof_key[1])
                if weight_per_m > 0:
                    profile_name = u"{}x{} (Радиусный, {} кг/м.п.)".format(
                        prof_key[0], prof_key[1], str(weight_per_m).replace('.', ','))
                curved_items = sorted(curved_profiles[prof_key], key=lambda x: -x[0])
                zone_total_weight = 0.0
                
                for j, item in enumerate(curved_items):
                    arc_len = item[0]
                    radius = item[1]
                    count = item[2]
                    total_weight = 0.0
                    
                    if weight_per_m > 0:
                        total_weight = round((arc_len / 1000.0) * weight_per_m * count, 2)
                        zone_total_weight += total_weight
                        profiles_total_weight += total_weight
                        
                    if j == 0:
                        sh.Cells(row, 1).Value = profile_name
                        cell_colors = {
                            (20, 20): 9221330,  
                            (25, 25): 9498256,  
                            (25, 40): 15128749, 
                            (40, 25): 15128749, 
                            (40, 40): 13882323  
                        }
                        if prof_key in cell_colors:
                            sh.Cells(row, 1).Interior.Color = cell_colors[prof_key]
                    else:
                        sh.Cells(row, 1).Value = ""
                        
                    sh.Cells(row, 2).Value = arc_len
                    sh.Cells(row, 3).Value = radius
                    sh.Cells(row, 4).Value = count
                    sh.Cells(row, 5).Value = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m > 0 else u"Нет в базе"
                    row += 1
                    
                total_row = None
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sh.Cells(row, 4).Value = u"Итого:"
                    sh.Cells(row, 4).Font.Bold = True
                    sh.Cells(row, 5).Value = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    sh.Cells(row, 5).Font.Bold = True
                    total_row = row
                    row += 1
                    
                header_range = sh.Range(sh.Cells(header_row, 1), sh.Cells(header_row, 5))
                header_range.Font.Bold = True
                for edge in (7, 8, 9, 10, 11):
                    try:
                        header_range.Borders(edge).LineStyle = xlContinuous
                        header_range.Borders(edge).Weight = xlThick
                    except:
                        pass
                        
                if total_row:
                    total_range = sh.Range(sh.Cells(total_row, 4), sh.Cells(total_row, 5))
                    for edge in (7, 8, 9, 10, 11):
                        try:
                            total_range.Borders(edge).LineStyle = xlContinuous
                            total_range.Borders(edge).Weight = xlThick
                        except:
                            pass

            for grp_key in sorted_sheets:
                if row > 1:
                    row += 1

                header_row = row
                sh.Cells(row, 1).Value = u"Материал"
                sh.Cells(row, 2).Value = u"Толщина"
                sh.Cells(row, 3).Value = u"Размер"
                sh.Cells(row, 4).Value = u"Количество"
                sh.Cells(row, 5).Value = u"Вес"
                row += 1

                weight_per_m2 = SHEET_MATERIALS.get(grp_key, 0.0)
                material_name = sheet_label(grp_key, weight_per_m2)
                sheet_items = sorted(sheets[grp_key], key=lambda x: -(x[0] * x[1]))
                zone_total_weight = 0.0

                for j, item in enumerate(sheet_items):
                    sheet_w = item[0]
                    sheet_l = item[1]
                    count = item[2]
                    total_weight = 0.0

                    if weight_per_m2 > 0:
                        total_weight = round((sheet_w / 1000.0) * (sheet_l / 1000.0) * weight_per_m2 * count, 2)
                        zone_total_weight += total_weight

                    if j == 0:
                        sh.Cells(row, 1).Value = material_name
                    else:
                        sh.Cells(row, 1).Value = ""

                    sh.Cells(row, 2).Value = grp_key[1]
                    sh.Cells(row, 3).Value = u"{}x{}".format(sheet_w, sheet_l)
                    sh.Cells(row, 4).Value = count
                    sh.Cells(row, 5).Value = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m2 > 0 else u"Нет в базе"
                    row += 1

                total_row = None
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sheet_totals[grp_key] = sheet_totals.get(grp_key, 0.0) + zone_total_weight
                    sh.Cells(row, 4).Value = u"Итого:"
                    sh.Cells(row, 4).Font.Bold = True
                    sh.Cells(row, 5).Value = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    sh.Cells(row, 5).Font.Bold = True
                    total_row = row
                    row += 1

                header_range = sh.Range(sh.Cells(header_row, 1), sh.Cells(header_row, 5))
                header_range.Font.Bold = True
                for edge in (7, 8, 9, 10, 11):
                    try:
                        header_range.Borders(edge).LineStyle = xlContinuous
                        header_range.Borders(edge).Weight = xlThick
                    except:
                        pass

                if total_row:
                    total_range = sh.Range(sh.Cells(total_row, 4), sh.Cells(total_row, 5))
                    for edge in (7, 8, 9, 10, 11):
                        try:
                            total_range.Borders(edge).LineStyle = xlContinuous
                            total_range.Borders(edge).Weight = xlThick
                        except:
                            pass

            for grp_key in sorted_curved_sheets:
                if row > 1:
                    row += 1

                header_row = row
                sh.Cells(row, 1).Value = u"Материал"
                sh.Cells(row, 2).Value = u"Толщина"
                sh.Cells(row, 3).Value = u"Размер (развёртка)"
                sh.Cells(row, 4).Value = u"Радиус (внешн.)"
                sh.Cells(row, 5).Value = u"Количество"
                sh.Cells(row, 6).Value = u"Вес"
                row += 1

                weight_per_m2 = SHEET_MATERIALS.get(grp_key, 0.0)
                material_name = curved_sheet_label(grp_key, weight_per_m2)
                sheet_items = sorted(curved_sheets[grp_key], key=lambda x: -(x[0] * x[1]))
                zone_total_weight = 0.0

                for j, item in enumerate(sheet_items):
                    sheet_w = item[0]
                    sheet_l = item[1]
                    sheet_r = item[2]
                    count = item[3]
                    total_weight = 0.0

                    if weight_per_m2 > 0:
                        total_weight = round((sheet_w / 1000.0) * (sheet_l / 1000.0) * weight_per_m2 * count, 2)
                        zone_total_weight += total_weight

                    if j == 0:
                        sh.Cells(row, 1).Value = material_name
                    else:
                        sh.Cells(row, 1).Value = ""

                    sh.Cells(row, 2).Value = grp_key[1]
                    sh.Cells(row, 3).Value = u"{}x{}".format(sheet_w, sheet_l)
                    sh.Cells(row, 4).Value = sheet_r
                    sh.Cells(row, 5).Value = count
                    sh.Cells(row, 6).Value = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m2 > 0 else u"Нет в базе"
                    row += 1

                total_row = None
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sheet_totals[grp_key] = sheet_totals.get(grp_key, 0.0) + zone_total_weight
                    sh.Cells(row, 5).Value = u"Итого:"
                    sh.Cells(row, 5).Font.Bold = True
                    sh.Cells(row, 6).Value = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    sh.Cells(row, 6).Font.Bold = True
                    total_row = row
                    row += 1

                header_range = sh.Range(sh.Cells(header_row, 1), sh.Cells(header_row, 6))
                header_range.Font.Bold = True
                for edge in (7, 8, 9, 10, 11):
                    try:
                        header_range.Borders(edge).LineStyle = xlContinuous
                        header_range.Borders(edge).Weight = xlThick
                    except:
                        pass

                if total_row:
                    total_range = sh.Range(sh.Cells(total_row, 5), sh.Cells(total_row, 6))
                    for edge in (7, 8, 9, 10, 11):
                        try:
                            total_range.Borders(edge).LineStyle = xlContinuous
                            total_range.Borders(edge).Weight = xlThick
                        except:
                            pass

            full_range = sh.Range(sh.Cells(1, 1), sh.Cells(row, 6))
            full_range.HorizontalAlignment = xlLeft
            full_range.VerticalAlignment = xlCenter
            
            summary_rows = build_summary(profiles_total_weight, sheet_totals)
            if summary_rows:
                for idx, summary_item in enumerate(summary_rows):
                    summary_row = idx + 1
                    sh.Cells(summary_row, 8).Value = summary_item[0]
                    sh.Cells(summary_row, 8).Font.Bold = summary_item[2]
                    sh.Cells(summary_row, 9).Value = u"{} кг".format(str(summary_item[1]).replace('.', ','))
                    sh.Cells(summary_row, 9).Font.Bold = summary_item[2]

                overall_range = sh.Range(sh.Cells(1, 8), sh.Cells(len(summary_rows), 9))
                overall_range.HorizontalAlignment = xlLeft
                overall_range.VerticalAlignment = xlCenter
                for edge in (7, 8, 9, 10, 11):
                    try:
                        overall_range.Borders(edge).LineStyle = xlContinuous
                        overall_range.Borders(edge).Weight = xlThick
                    except:
                        pass
            
            sh.Columns.AutoFit()
            rs.MessageBox(u"Таблица успешно сформирована и открыта в Excel!", 0, u"Готово")
            return
    except Exception as e:
        print("Прямой вывод в Excel недоступен: " + str(e))
        pass

    rs.MessageBox(u"Оформление (жирный шрифт, рамки) недоступно (требуется Windows и Excel). Таблица будет сохранена как обычный CSV-файл.", 0, u"Внимание")
    
    filename = rs.SaveFileName(u"Сохранить таблицу деталей (без оформления)", "CSV Files (*.csv)|*.csv||")
    if not filename:
        return

    try:
        with codecs.open(filename, 'w', encoding='utf-8-sig') as f:
            profiles_total_weight = 0.0
            sheet_totals = {}
            
            for i, prof_key in enumerate(sorted_profiles):
                if i > 0:
                    f.write(u"\n")
                
                f.write(u"Профиль;Длина;Количество;Вес\n")
                weight_per_m = PROFILE_WEIGHTS.get(prof_key, 0.0)
                profile_name = u"{}x{}".format(prof_key[0], prof_key[1])
                if weight_per_m > 0:
                    profile_name += u" ({} кг/м.п.)".format(str(weight_per_m).replace('.', ','))
                lengths = sorted(profiles[prof_key], key=lambda x: -x[0])
                zone_total_weight = 0.0
                
                for j, item in enumerate(lengths):
                    length = item[0]
                    count = item[1]
                    total_weight = 0.0
                    
                    if weight_per_m > 0:
                        total_weight = round((length / 1000.0) * weight_per_m * count, 2)
                        zone_total_weight += total_weight
                        profiles_total_weight += total_weight
                        
                    weight_str = u"{} кг".format(str(total_weight).replace('.', ',')) if total_weight > 0 else u"Нет в базе"
                    
                    if j == 0:
                        f.write(u"{};{};{};{}\n".format(profile_name, length, count, weight_str))
                    else:
                        f.write(u";{};{};{}\n".format(length, count, weight_str))
                
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    zone_weight_str = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    f.write(u";;Итого:;{}\n".format(zone_weight_str))
                    
            for i, prof_key in enumerate(sorted_curved_profiles):
                f.write(u"\n")
                
                f.write(u"Профиль;Длина;Радиус (внешн.);Количество;Вес\n")
                weight_per_m = PROFILE_WEIGHTS.get(prof_key, 0.0)
                profile_name = u"{}x{} (Радиусный)".format(prof_key[0], prof_key[1])
                if weight_per_m > 0:
                    profile_name = u"{}x{} (Радиусный, {} кг/м.п.)".format(
                        prof_key[0], prof_key[1], str(weight_per_m).replace('.', ','))
                curved_items = sorted(curved_profiles[prof_key], key=lambda x: -x[0])
                zone_total_weight = 0.0
                
                for j, item in enumerate(curved_items):
                    arc_len = item[0]
                    radius = item[1]
                    count = item[2]
                    total_weight = 0.0
                    
                    if weight_per_m > 0:
                        total_weight = round((arc_len / 1000.0) * weight_per_m * count, 2)
                        zone_total_weight += total_weight
                        profiles_total_weight += total_weight
                        
                    weight_str = u"{} кг".format(str(total_weight).replace('.', ',')) if total_weight > 0 else u"Нет в базе"
                    
                    if j == 0:
                        f.write(u"{};{};{};{};{}\n".format(profile_name, arc_len, radius, count, weight_str))
                    else:
                        f.write(u";{};{};{};{}\n".format(arc_len, radius, count, weight_str))
                
                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    zone_weight_str = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    f.write(u";;;Итого:;{}\n".format(zone_weight_str))
            
            for grp_key in sorted_sheets:
                f.write(u"\n")

                f.write(u"Материал;Толщина;Размер;Количество;Вес\n")
                weight_per_m2 = SHEET_MATERIALS.get(grp_key, 0.0)
                material_name = sheet_label(grp_key, weight_per_m2)
                sheet_items = sorted(sheets[grp_key], key=lambda x: -(x[0] * x[1]))
                zone_total_weight = 0.0

                for j, item in enumerate(sheet_items):
                    sheet_w = item[0]
                    sheet_l = item[1]
                    count = item[2]
                    total_weight = 0.0

                    if weight_per_m2 > 0:
                        total_weight = round((sheet_w / 1000.0) * (sheet_l / 1000.0) * weight_per_m2 * count, 2)
                        zone_total_weight += total_weight

                    weight_str = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m2 > 0 else u"Нет в базе"
                    size_str = u"{}x{}".format(sheet_w, sheet_l)

                    if j == 0:
                        f.write(u"{};{};{};{};{}\n".format(material_name, grp_key[1], size_str, count, weight_str))
                    else:
                        f.write(u";{};{};{};{}\n".format(grp_key[1], size_str, count, weight_str))

                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sheet_totals[grp_key] = sheet_totals.get(grp_key, 0.0) + zone_total_weight
                    zone_weight_str = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    f.write(u";;;Итого:;{}\n".format(zone_weight_str))

            for grp_key in sorted_curved_sheets:
                f.write(u"\n")

                f.write(u"Материал;Толщина;Размер (развёртка);Радиус (внешн.);Количество;Вес\n")
                weight_per_m2 = SHEET_MATERIALS.get(grp_key, 0.0)
                material_name = curved_sheet_label(grp_key, weight_per_m2)
                sheet_items = sorted(curved_sheets[grp_key], key=lambda x: -(x[0] * x[1]))
                zone_total_weight = 0.0

                for j, item in enumerate(sheet_items):
                    sheet_w = item[0]
                    sheet_l = item[1]
                    sheet_r = item[2]
                    count = item[3]
                    total_weight = 0.0

                    if weight_per_m2 > 0:
                        total_weight = round((sheet_w / 1000.0) * (sheet_l / 1000.0) * weight_per_m2 * count, 2)
                        zone_total_weight += total_weight

                    weight_str = u"{} кг".format(str(total_weight).replace('.', ',')) if weight_per_m2 > 0 else u"Нет в базе"
                    size_str = u"{}x{}".format(sheet_w, sheet_l)

                    if j == 0:
                        f.write(u"{};{};{};{};{};{}\n".format(material_name, grp_key[1], size_str, sheet_r, count, weight_str))
                    else:
                        f.write(u";{};{};{};{};{}\n".format(grp_key[1], size_str, sheet_r, count, weight_str))

                if zone_total_weight > 0:
                    zone_total_weight = round(zone_total_weight, 2)
                    sheet_totals[grp_key] = sheet_totals.get(grp_key, 0.0) + zone_total_weight
                    zone_weight_str = u"{} кг".format(str(zone_total_weight).replace('.', ','))
                    f.write(u";;;;Итого:;{}\n".format(zone_weight_str))

            summary_rows = build_summary(profiles_total_weight, sheet_totals)
            if summary_rows:
                f.write(u"\n")
                for summary_item in summary_rows:
                    f.write(u";;{};{} кг\n".format(
                        summary_item[0], str(summary_item[1]).replace('.', ',')))
                
        rs.MessageBox(u"Таблица успешно сохранена как CSV-файл:\n" + filename, 0, u"Готово")
    except Exception as e:
        rs.MessageBox(u"Ошибка при сохранении файла: " + str(e), 0, u"Ошибка")

if __name__ == "__main__":
    calculate_parts()

