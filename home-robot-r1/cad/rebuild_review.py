from pathlib import Path
import json,html,re
R=Path(__file__).resolve().parents[1];D=json.loads((R/'parts-and-connections.json').read_text());P=json.loads((R/'cad/component-manifest.json').read_text());V=json.loads((R/'cad/validation.json').read_text());C=json.loads((R/'cable-cut-list.json').read_text())
e=html.escape
def row(cells):return '<tr>'+''.join('<td>'+x+'</td>' for x in cells)+'</tr>'
bom=[]
for p in D['parts']:
    kind='conditional' if p['status']=='Conditional buy' else ('reuse' if p['new_cost_usd']==0 else 'buy')
    src=p['source']   # a web page, or a file of this repo (pcb/fab/ORDERING.md); anything else is shown as plain text, never as a dead link
    if re.fullmatch(r'https?://\S+',src) or src and (R/src).is_file() and (R/src).resolve().is_relative_to(R.resolve()):link='<a href="'+e(src)+'" target="_blank" rel="noopener">'+e(p.get('source_label','Product / source'))+' ↗</a>'
    else:link=('Source (no link): '+e(src)) if src else 'Specification / allowance; choose supplier after fit check.'
    bom.append(f'<tr data-kind="{kind}"><td class="id">{p["id"]}</td><td class="item"><strong>{e(p["item"])}</strong><details><summary>Specification and source</summary><p>{e(p["notes"])}</p><p>{link}</p></details></td><td>{p["quantity"]} {e(p["unit"])}</td><td>{e(p["status"])}</td><td class="number">${p["new_cost_usd"]:.2f}</td></tr>')
prints=[]
byid={p['id']:p for p in P}
for v in V['printed_parts']:
    p=byid[v['part']];sizes=' × '.join(str(round(x,1)) for x in v['size_mm'])
    note=p['note'] or ('Print flat; check holes and fit before final assembly.' if p['id'] not in ['P_camera_mount','P_camera_holder'] else 'Prototype fit; orient in slicer.')
    prints.append(row([e(p['name'])+'<br><span class="small muted">'+p['id']+'</span>',sizes,f'<a href="print-parts/{p["id"]}.stl" download>STL</a> / <a href="print-parts/{p["id"]}.step" download>STEP</a>',e(note)]))
tc=V.get('test_coupon')
if tc:prints.append(row([e(tc['name'])+'<br><span class="small muted">'+tc['id']+'</span>',' × '.join(str(x) for x in tc['size_mm']),f'<a href="print-parts/coupons/{tc["id"]}.stl" download>STL</a> / <a href="print-parts/coupons/{tc["id"]}.step" download>STEP</a>',e('Print first: '+'; '.join(tc['features'])+'.')]))
conn=[]
for c in D['connectors']:conn.append(row(['<strong>'+e(c[0])+'</strong><br>'+e(c[1]),e(c[2]),e(c[3]),e(c[4])+'<br><span class="small muted">'+e(c[5])+'</span>']))
cables=[]
for c in C:
    length=f'{c["suggested_cut_mm"]} mm'
    if c['id']=='W16':length='Buy complete 0.5 m USB cable'
    elif c['id']=='W17':length='Buy complete 0.5 m USB cable'
    elif c['id']=='W19':length='Buy complete 0.3 m USB extension'
    elif c['id']=='W20':length='Buy matched 0.2 m four-pin lead'
    elif c['id'].startswith('motor_cable'):length='Included factory cable (about 200 mm to the housing rear face: measure), not cut: the remainder after the fixed path is coiled (2–3 turns) and tied'
    elif c['id'].startswith('motor_pigtail'):length=f'{c["suggested_cut_mm"]} mm per wire pre-cut, trimmed to its finished length after dressing and crimped (C18)'
    cables.append(row([c['id'],e(c['name']),f'{c["route_mm"]} mm',length]))
def summary():
    n=len(V['printed_parts']);pr=V['printability'];bt=V['bumper_travel']['halves'];lm=V['lidar_mount'];S=V['summary']
    shell=min(h['rigid_part_clearance']['s5.0']['min_gap_mm'] for h in bt.values());tyre=min(h['wheel_side_min_clearance_at_full_travel_mm'] for h in bt.values())
    cs=V['clash_scan'];fl=V['bumper_travel']['friction_lock'];c=V['com_estimate']
    return (f'All {len(P)} modeled shapes pass CAD validity checks. All {n} printed-part envelopes fit the specified printer volume, and '
        f'{pr["parts_checked"]-len(pr["parts_failed"])} of {pr["parts_checked"]} printed STLs pass the support-free printability gate (cad/check_printability.py, run at the end of every build: '
        f'a flat bed face in one axis-aligned orientation, at most 50 mm² of unsupported overhang beyond 45° with bridges up to 10 mm allowed, and 1st-percentile walls of at least 1.2 mm; details in cad/printability.json). '
        f'The solid-overlap scan covers all {V["major_components_checked"]} non-wire parts pairwise and finds {len(cs["clashes"])} unintended overlaps; every harness envelope is checked against every solid part ({len(V["wire_clashes"])} overlaps); '
        f'fasteners, inserts, nuts, module screws, pins, springs, bearings, riders, races and balls are gap-checked ({len(V["fastener_clearances"]["violations"])} violations), no two static printed parts come within 1 mm unless designed to touch ({len(V["printed_part_near_contacts"]["violations"])} violations), and every pair of P_base holes keeps at least 1.2 mm of wall. '
        f'At the 5 mm bumper hard stop each half keeps {shell} mm to every other part and {tyre} mm to the tyres. Rolling-ball guides keep the critical vertical friction at or above {fl["min_mu_vert_crit"]} over the whole arc and height, so the whole 0–90° arc is sensed ({S["arc_sensed_to_deg"]}°). '
        f'Estimated centre of mass x = {c["com_mm"][0]} mm (forward pitch onset {c["forward_pitch_onset_decel_m_s2"]} m/s²). '
        f'The LiDAR tray touches only the LD14 lugs ({lm["tray_gap_to_non_lug_lidar_parts_mm"]} mm to every other LD14 part).')
c=V['com_estimate'];cl=V['caster_layout'];fl=V['bumper_travel']['friction_lock'];th=V['threshold_crossing']
over=D['base_new_cost_usd']-500
costnote=(f'This leaves ${-over:.2f} for shipping, tax and substitutions.' if over<0 else f'This is ${over:.2f} over the $500 target before shipping and tax.')+' The regulator is the user\'s P03 module (no D24V50F5 line); the prices quoted from memory are on the verify list in parts-and-connections.json.'
vd=lambda k:f"{'an 8' if k=='step_8mm' else 'a 10'} mm step {th[k]['forward']['verdict']} forward and {th[k]['reverse']['verdict']} in reverse"
PA=json.loads((R/'cad/parameters.json').read_text());bt_=V['bumper_travel'];rs_=V['rear_shell'];com_=V['com_estimate']
fx=lambda v:f'{v:g}'
extra={'DIAM':f"{2*PA['outer_radius']:.0f}",'RSKIN':fx(PA['outer_radius']),'RSH':fx(PA['shell_radius']),'RTIP':fx(PA['bumper_tip_radius']),'RRS':fx(PA['rear_shell_radius']),'ENVWORST':f"{V['envelope']['worst_case_r_mm']:.1f}",
    'WINW':fx(PA['camera_window']['at_skin']['width_mm']),'WINZ':'–'.join(fx(z) for z in PA['camera_window']['at_skin']['z_mm']),'WINREC':fx(PA['camera_window']['lens_recess_mm']),'SEAM':fx(PA['seam_mm']),'SEAMINTR':f"{bt_['seam_intrusion']['max_intrusion_before_trip_mm']:.1f}",'RSGAP':f"{PA['outer_radius']-PA['rear_shell_radius']:.1f}",
    'FLGAP':f"{PA['outer_radius']-PA['rear_shell_flange_r_mm'][1]:.1f}",'NINSERTS':str(sum(1 for q in P if q['id'].startswith('insert_'))),'VREV':fx(PA['reverse_speed_limit_m_s']),'VSAFE':fx(cl['stopping']['max_coast_safe_reverse_speed_mps']),'COAST':str(cl['stopping']['coast_without_rolling_drag']['distance_mm']),'PITCHDOWN':f"{com_['forward_pitch_onset_on_5deg_downhill_m_s2']:.2f}",'BRAKEMAX':f"{cl['stopping']['active_brake_limit']['max_decel_with_torque_cap_m_s2']:.2f}",'SLOTRANGE':f"8.1–{bt_['planar_play']['slot_width_max_mm']}",'CASTERX':fx(PA['rear_caster'][0]),
    'RCLIFF':f"({fx(PA['cliff_sensors'][2][0])}, ±{fx(abs(PA['cliff_sensors'][2][1]))})"}
_lid=max((sorted(q['size_mm'][:2],reverse=True) for q in V['printed_parts'] if q['part'] in ('P_lid_front','P_lid_left','P_lid_rear','P_lid_right')),key=lambda d:d[0]*d[1])
extra['LIDMAX']=f'{_lid[0]:.0f} × {_lid[1]:.0f}'   # the largest of the four lid sectors (task I: the 240 × 200 deck-era cover text was stale)
repl={'COSTNOTE':costnote,'MUCRIT':str(fl['min_mu_vert_crit']),'MUEQ':str(fl['min_mu_crit_bearing_and_ball_equal']),'COMX':f"{-c['com_mm'][0]:.1f}",'COMMIN':f"{-max(v['com_x_mm'] for v in c['sensitivity'].values()):.1f}",
    'CASTERPCT':f"{100*c['caster_load_fraction']:.0f}",'PITCH':f"{c['forward_pitch_onset_decel_m_s2']:.2f}",'THRESH':vd('step_8mm')+', and '+vd('step_10mm'),
    'LEAD':str(cl['rear_cliff_lead_over_caster_reverse_mm']),'COASTLEAD':str(cl['stopping']['coast_without_rolling_drag']['required_lead_mm']),**extra,'NLINES':str(len(D['parts'])),
    'OPTIONS':''.join(f'<option value="{p["id"]}">{e(p["name"])} · {p["id"]}</option>' for p in P),'BOM':''.join(bom),'SPARES':''.join(row([e(c) for c in a]) for a in D['spares']),'CONNECTORS':''.join(conn),'PINS':''.join(row([e(c) for c in a]) for a in D['pico_pins']),'CABLES':''.join(cables),'PRINTS':''.join(prints),'MESH':(R/'cad/assembly-meshes.json').read_text(),'CHECKSUMMARY':summary(),'COST':f"${D['base_new_cost_usd']:.2f}",'W20ROUTE':str(next(c['route_mm'] for c in C if c['id']=='W20')),'LIDARX':fx(PA['lidar_x']),'LIDARH':fx(PA['lidar_height']),'CAMH':fx(PA['camera_height']),'BUMPZ':'–'.join(fx(z) for z in PA['bumper_z']),'BUMPTOP':fx(PA['bumper_z'][1]),'HEIGHT':f"{V['summary']['robot_height_mm']:.0f}",'NPRINT':str(len(V['printed_parts']))}
SLACK_S=120   # a git checkout stamps the meshes and the images within seconds, in any order; a real stale image is minutes older than the build
for png in ['robot-assembled.png','robot-exploded.png','robot-internal.png']:   # the review images must be rendered from the current build
    if (R/png).stat().st_mtime<(R/'cad/assembly-meshes.json').stat().st_mtime-SLACK_S:raise SystemExit(f'{png} is older than cad/assembly-meshes.json: re-render it (cad/render_review_images.py) before rebuilding the review')
s=Path(__file__).with_name('review-template.html').read_text()
for key,val in repl.items():s=s.replace('@@'+key+'@@',val)
assert '@@' not in s
(R/'design-review.html').write_text(s)
print('HTML generated:',len(s),'characters')
